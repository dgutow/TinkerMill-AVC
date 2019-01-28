# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'cosmos/config/config_parser'
require 'cosmos/system/system'
require 'fileutils'

module Cosmos

  # Handles writing message logs to a file
  class MessageLog

    # @return [String] The name of the message log file. Empty string until the
    #   write or start methods are called at which point it is set to the
    #   filename. Retains the last filename even after stop is called.
    attr_reader :filename

    # @param tool_name [String] The name of the tool creating the message log.
    #   This will be inserted into the message log file name to help identify it.
    # @param log_dir [String] The filesystem path to store the message log file.
    def initialize(tool_name, log_dir = nil)
      @tool_name = tool_name
      @log_dir = ConfigParser.handle_nil(log_dir)
      @log_dir = System.paths['LOGS'] unless @log_dir
      @filename = ''
      @file = nil
      @start_time = nil
      @mutex = Mutex.new
    end

    # Ensures the log file is opened and ready to write. It then writes the
    # message to the log and flushes it to force the write.
    #
    # @param message [String] Message to write to the log
    def write(message)
      @mutex.synchronize do
        if @file.nil? or @file.closed? or (not File.exist?(@filename))
          start(false)
        end

        @file.write(message)
      end
    end

    # Closes the message log and marks it read only
    def stop(take_mutex = true)
      @mutex.lock if take_mutex
      if @file and not @file.closed?
        @file.close
        Cosmos.set_working_dir do
          File.chmod(0444, @filename)
        end
      end
      @mutex.unlock if take_mutex
    end

    # Creates a new message log and sets the filename
    def start(take_mutex = true)
      @mutex.lock if take_mutex
      # Prevent starting files too fast
      sleep(0.1) until !File.exist?(File.join(@log_dir, File.build_timestamped_filename([@tool_name, 'messages'])))
      stop(false)
      Cosmos.set_working_dir do
        @filename = File.join(@log_dir, File.build_timestamped_filename([@tool_name, 'messages']))
        @file = File.open(@filename, 'a')
      end
      @mutex.unlock if take_mutex
    end

  end # class MessageLog

end # module Cosmos
