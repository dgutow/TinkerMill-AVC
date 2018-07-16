# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'cosmos/script/script'
require 'cosmos/gui/choosers/combobox_chooser'

$cmd_tlm_gui_window = nil

class Qt::Dialog
  def exec(*args)
    Cosmos.play_wav_file(Cosmos.data_path('message.wav')) if Cosmos::System.sound
    method_missing(:exec, *args)
  end
end
class Qt::MessageBox
  def exec(*args)
    Cosmos.play_wav_file(Cosmos.data_path('message.wav')) if Cosmos::System.sound
    method_missing(:exec, *args)
  end
  def self.critical(parent, title, text,
                    buttons = Qt::MessageBox::Ok,
                    defaultButton = Qt::MessageBox::NoButton)
    # Windows overrides critical dialogs with its own alert sound
    Cosmos.play_wav_file(Cosmos.data_path('critical.wav')) if Cosmos::System.sound
    super(parent,title,text,buttons,defaultButton)
  end
  def self.information(parent, title, text,
                       buttons = Qt::MessageBox::Ok,
                       defaultButton = Qt::MessageBox::NoButton)
    Cosmos.play_wav_file(Cosmos.data_path('information.wav')) if Cosmos::System.sound
    super(parent,title,text,buttons,defaultButton)
  end
  def self.question(parent, title, text,
                    buttons = Qt::MessageBox::Ok,
                    defaultButton = Qt::MessageBox::NoButton)
    Cosmos.play_wav_file(Cosmos.data_path('question.wav')) if Cosmos::System.sound
    super(parent,title,text,buttons,defaultButton)
  end
  def self.warning(parent, title, text,
                   buttons = Qt::MessageBox::Ok,
                   defaultButton = Qt::MessageBox::NoButton)
    # Windows overrides warning dialogs with its own alert sound
    Cosmos.play_wav_file(Cosmos.data_path('warning.wav')) if Cosmos::System.sound
    super(parent,title,text,buttons,defaultButton)
  end
end

class Qt::InputDialog
  def self.getText(*args)
    Cosmos.play_wav_file(Cosmos.data_path('input.wav')) if Cosmos::System.sound
    super(*args)
  end
end

module Cosmos

  # Cosmos script changes to handle hazardous commands and prompts
  old_verbose = $VERBOSE; $VERBOSE = nil
  module Script
    private

    @@qt_boolean = Qt::Boolean.new

    def _get_main_thread_gui
      result = nil
      Qt.execute_in_main_thread(true, 0.05) do
        result = yield(get_cmd_tlm_gui_window())
      end
      return result
    end

    def save_file_dialog(directory = Cosmos::USERPATH, message = "Save File", filter = "All Files (*.*)")
      _get_main_thread_gui {|window| Qt::FileDialog.getSaveFileName(window, message, directory, filter) }
    end

    def open_file_dialog(directory = Cosmos::USERPATH, message = "Open File", filter = "All Files (*.*)")
      _get_main_thread_gui {|window| Qt::FileDialog.getOpenFileName(window, message, directory, filter) }
    end

    def open_files_dialog(directory = Cosmos::USERPATH, message = "Open File(s)", filter = "All Files (*.*)")
      _get_main_thread_gui {|window| Qt::FileDialog.getOpenFileNames(window, message, directory, filter) }
    end

    def open_directory_dialog(directory = Cosmos::USERPATH, message = "Open Directory")
      _get_main_thread_gui {|window| Qt::FileDialog.getExistingDirectory(window, message, directory) }
    end

    def ask_string(question, blank_or_default = false, password = false)
      answer = ''
      if blank_or_default != true && blank_or_default != false
        default = blank_or_default.to_s
        allow_blank = false
      else
        default = ''
        allow_blank = blank_or_default
      end
      loop do
        canceled = false
        _get_main_thread_gui do |window|
          # Create a special mutable QT variable that can return what button was pressed
          if password
            answer = Qt::InputDialog::getText(window, "Ask", question, Qt::LineEdit::Password, default, @@qt_boolean)
          else
            answer = Qt::InputDialog::getText(window, "Ask", question, Qt::LineEdit::Normal, default, @@qt_boolean)
          end
          # @@qt_boolean is nil if the user presses cancel in the dialog
          # Note that it is not actually nil, just the nil? method returns true
          canceled = @@qt_boolean.nil?
        end
        if canceled
          Logger.warn "User pressed 'Cancel' for '#{question}'"
          prompt_for_script_abort()
        end
        # If the answer wasn't nil and contains a value we break
        break if allow_blank or (not answer.nil? and answer.strip.length != 0)
      end

      if password
        Logger.info "User responded to '#{question}'"
      else
        Logger.info "User entered '#{answer}' for '#{question}'"
      end
      return answer.to_s
    end

    def prompt_dialog_box(title, message)
      result = nil
      _get_main_thread_gui do |window|
        msg = Qt::MessageBox.new(window)
        msg.setIcon(Qt::MessageBox::Question)
        msg.setText(message)
        msg.setWindowTitle(title)
        msg.setStandardButtons(Qt::MessageBox::Yes | Qt::MessageBox::No)
        msg.setDefaultButton(Qt::MessageBox::Yes)
        case msg.exec()
        when Qt::MessageBox::Yes
          Logger.info "User pressed 'Yes' for '#{message}'"
          result = true
        when Qt::MessageBox::No
          Logger.info "User pressed 'No' for '#{message}'"
          result = false
        end
        msg.dispose
      end
      return result
    end

    def prompt_for_hazardous(target_name, cmd_name, hazardous_description)
      message = "Warning: Command #{target_name} #{cmd_name} is Hazardous. "
      message << "\n#{hazardous_description}\n" if hazardous_description
      message << "Send?"
      return prompt_dialog_box('Hazardous Command', message)
    end

    def prompt_for_script_abort
      return true # Aborted - Don't retry
    end

    def prompt_to_continue(string)
      loop do
        stop = false
        _get_main_thread_gui do |window|
          result = Qt::MessageBox::question(window,
                                            "COSMOS",
                                            "#{string}\n\nOK to Continue?",
                                            Qt::MessageBox::Ok | Qt::MessageBox::Cancel,
                                            Qt::MessageBox::Ok)
          if result == Qt::MessageBox::Ok
            Logger.info "User pressed 'Ok' for '#{string}'"
          else
            stop = true
            Logger.warn "User pressed 'Cancel' for '#{string}'"
          end
        end
        if stop
          prompt_for_script_abort()
        else
          break
        end
      end
    end

    def prompt_message_box(string, buttons)
      loop do
        result = nil
        _get_main_thread_gui do |window|
          msg = Qt::MessageBox.new(window)
          msg.setText(string)
          msg.setWindowTitle("Message Box")
          # Check if the last parameter is false which means they don't want
          # the Cancel button to be displayed
          if buttons[-1] == false
            buttons[0..-2].each {|text| msg.addButton(text, Qt::MessageBox::AcceptRole)}
          else
            buttons.each {|text| msg.addButton(text, Qt::MessageBox::AcceptRole)}
            msg.addButton("Cancel", Qt::MessageBox::RejectRole)
          end
          msg.exec()
          if msg.clickedButton.text == "Cancel"
            Logger.warn "User pressed 'Cancel' for '#{string}'"
          else
            Logger.info "User pressed '#{msg.clickedButton.text}' for '#{string}'"
          end
          result = msg.clickedButton.text
          msg.dispose
        end
        if result == "Cancel"
          prompt_for_script_abort()
        else
          return result
        end
      end
    end

    def prompt_vertical_message_box(string, buttons)
      loop do
        result = buttons[0].clone
        Qt.execute_in_main_thread(true, 0.05) do
          dialog = _build_dialog(string)

          # Check if the last parameter is false which means they don't want
          # the Cancel button to be displayed
          if buttons[-1] == false
            cancel = false
            display_buttons = buttons[0..-2]
          else
            cancel = true
            display_buttons = buttons
          end
          button_layout = Qt::VBoxLayout.new
          button_layout.setContentsMargins(11,11,11,11)
          display_buttons.each do |button_text|
            button = Qt::PushButton.new(button_text)
            button.connect(SIGNAL('clicked()')) do
              result.replace(button_text)
              dialog.accept()
            end
            button_layout.addWidget(button)
          end
          dialog.layout.addLayout(button_layout)
          dialog.layout.addWidget(_build_dialog_buttons(dialog, false, cancel))
          result = "Cancel" unless _exec_dialog(dialog, string, result)
        end
        if result == "Cancel"
          prompt_for_script_abort()
        else
          return result
        end
      end
    end

    def prompt_combo_box(string, options)
      loop do
        result = options[0].clone
        Qt.execute_in_main_thread(true, 0.05) do
          dialog = _build_dialog(string)
          # Check if the last parameter is false which means they don't want
          # the Cancel button to be displayed
          if options[-1] == false
            cancel = false
            display_options = options[0..-2]
          else
            cancel = true
            display_options = options
          end
          chooser = ComboboxChooser.new(dialog, "Select:", display_options)
          chooser.setContentsMargins(11,11,11,11)
          chooser.sel_command_callback = lambda { |value| result.replace(value) }
          dialog.layout.addWidget(chooser)
          dialog.layout.addWidget(_build_dialog_buttons(dialog, true, cancel))
          result = "Cancel" unless _exec_dialog(dialog, string, result)
        end
        if result == "Cancel"
          prompt_for_script_abort()
        else
          return result
        end
      end
    end

    def _build_dialog(message)
      dialog = Qt::Dialog.new(get_cmd_tlm_gui_window())
      dialog.setWindowTitle("Message Box")
      layout = Qt::VBoxLayout.new
      layout.setContentsMargins(0,0,0,0)

      label = Qt::Label.new(message)
      label.setStyleSheet("background-color: white;")
      label.setMargin(11)
      layout.addWidget(label)
      dialog.setLayout(layout)
      dialog
    end

    def _exec_dialog(dialog, message, selection)
      result = true
      if dialog.exec() == Qt::Dialog::Accepted
        Logger.info "User selected '#{selection}' for '#{message}'"
      else
        Logger.warn "User pressed 'Cancel' for '#{message}'"
        result = false
      end
      dialog.dispose
      result
    end

    def _build_dialog_buttons(dialog, ok_button = true, cancel_button = true)
      button_layout = Qt::HBoxLayout.new
      if ok_button
        ok = Qt::PushButton.new("Ok")
        ok.connect(SIGNAL('clicked()')) do
          dialog.accept()
        end
        button_layout.addWidget(ok)
      end
      if cancel_button
        cancel = Qt::PushButton.new("Cancel")
        cancel.connect(SIGNAL('clicked()')) do
          dialog.reject()
        end
        button_layout.addWidget(cancel)
      end

      widget = Qt::Widget.new
      widget.setLayout(button_layout)
      widget
    end

    def get_scriptrunner_log_message(title_text = "Script Message Log Text Entry", prompt_text = 'Enter text to log to the script message log')
      answer = ""
      canceled = false
      _get_main_thread_gui do |window|
        # Create a special mutable QT variable that can return what button was pressed
        answer = Qt::InputDialog::getText(window, title_text, prompt_text, Qt::LineEdit::Normal, "", @@qt_boolean)
        # @@qt_boolean is nil if the user presses cancel in the dialog
        # Note that it is not actually nil, just the nil? method returns true
        canceled = @@qt_boolean.nil?
      end
      if canceled
        return nil
      else
        return answer.to_s
      end
    end

    def set_cmd_tlm_gui_window(window)
      $cmd_tlm_gui_window = window
    end

    def get_cmd_tlm_gui_window
      $cmd_tlm_gui_window
    end

  end # module Script
  $VERBOSE = old_verbose

end # module Cosmos
