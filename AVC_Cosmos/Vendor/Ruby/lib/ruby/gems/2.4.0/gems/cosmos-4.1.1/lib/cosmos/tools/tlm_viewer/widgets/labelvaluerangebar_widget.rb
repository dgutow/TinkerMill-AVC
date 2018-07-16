# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'cosmos/tools/tlm_viewer/widgets/widget'
require 'cosmos/tools/tlm_viewer/widgets/multi_widget'
require 'cosmos/tools/tlm_viewer/widgets/labelvalue_widget'
require 'cosmos/tools/tlm_viewer/widgets/rangebar_widget'

module Cosmos

  class LabelvaluerangebarWidget < Qt::Widget
    include Widget
    include MultiWidget

    def initialize(parent_layout, target_name, packet_name, item_name, low_value, high_value, value_type = :WITH_UNITS, characters = 12, width = 160, height = 25)
      super(target_name, packet_name, item_name, value_type)
      setLayout(Qt::HBoxLayout.new())
      layout.setSpacing(1)
      layout.setContentsMargins(0,0,0,0)
      @widgets << LabelvalueWidget.new(layout, target_name, packet_name, item_name, value_type, characters)
      @widgets << RangebarWidget.new(layout, target_name, packet_name, item_name, low_value, high_value, value_type, width, height)
      parent_layout.addWidget(self) if parent_layout
    end

    def self.takes_value?
      return true
    end
  end

end # module Cosmos
