# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'cosmos'
require 'cosmos/tools/tlm_viewer/widgets/widget'
require 'cosmos/tools/tlm_viewer/widgets/aging_widget'

module Cosmos

  class TextboxWidget < Qt::TextEdit
    include Widget
    include AgingWidget

    def initialize(parent_layout, target_name, packet_name, item_name, width = 200, height = 100, value_type = :CONVERTED)
      super(target_name, packet_name, item_name, value_type)
      setup_aging
      setFixedSize(width.to_i, height.to_i)
      setReadOnly(true)
      parent_layout.addWidget(self) if parent_layout
    end

    def format_value(data)
      data.to_s
    end

    def value=(data)
      scroll_pos = self.verticalScrollBar.value
      self.text = super(data, format_value(data))
      self.setColors(@foreground, @background)
      self.verticalScrollBar.value = scroll_pos
    end

    def process_settings
      super
      process_aging_settings
    end

  end

end # module Cosmos
