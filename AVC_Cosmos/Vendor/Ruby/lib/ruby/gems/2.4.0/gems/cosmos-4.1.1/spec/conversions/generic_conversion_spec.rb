# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'spec_helper'
require 'cosmos/conversions/generic_conversion'

module Cosmos

  describe GenericConversion do

    describe "initialize" do
      it "takes code_to_eval, converted_type and converted_bit_size" do
        gc = GenericConversion.new("10 / 2",:UINT,8)
        expect(gc.code_to_eval).to eql "10 / 2"
        expect(gc.converted_type).to eql :UINT
        expect(gc.converted_bit_size).to eql 8
      end

      it "complains about invalid converted_type" do
        expect { GenericConversion.new("",:MINE,8) }.to raise_error("Invalid type MINE")
      end
    end

    describe "call" do
      it "calls the code to eval and return the result" do
        gc = GenericConversion.new("10 / 2",:UINT,8)
        expect(gc.call(0,0,0)).to eql 5
      end
    end

    describe "to_s" do
      it "returns the code to eval" do
        expect(GenericConversion.new("10 / 2").to_s).to eql "10 / 2"
      end
    end
  end
end

