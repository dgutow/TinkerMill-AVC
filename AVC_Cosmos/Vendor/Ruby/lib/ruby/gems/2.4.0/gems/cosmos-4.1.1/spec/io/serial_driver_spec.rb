# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

if RUBY_ENGINE == 'ruby' or Gem.win_platform?

  require 'spec_helper'
  require 'cosmos/io/serial_driver'
  require 'cosmos/io/win32_serial_driver'

  module Cosmos

    describe SerialDriver do

      describe "instance" do
        it "enforces the parity to a known value" do
          expect { SerialDriver.new('COM1',9600,:BLAH) }.to raise_error(ArgumentError, "Invalid parity: BLAH")
        end
      end

      describe "close, closed?, write, read" do
        it "defers to the windows serial driver on windows" do
          allow(Kernel).to receive(:is_windows?).and_return(true)
          driver = double("Win32SerialDriver")
          expect(driver).to receive(:close)
          expect(driver).to receive(:closed?)
          expect(driver).to receive(:write)
          expect(driver).to receive(:read)
          allow(Win32SerialDriver).to receive(:new).and_return(driver)
          driver = SerialDriver.new('COM1',9600)
          driver.close
          driver.closed?
          driver.write("hi")
          driver.read
        end

        it "defers to the posix serial driver on nix" do
          class PosixSerialDriver
          end
          allow(Kernel).to receive(:is_windows?).and_return(false)
          driver = double("PosixSerialDriver")
          expect(driver).to receive(:close)
          expect(driver).to receive(:closed?)
          expect(driver).to receive(:write)
          expect(driver).to receive(:read)
          allow(PosixSerialDriver).to receive(:new).and_return(driver)
          driver = SerialDriver.new('COM1',9600)
          driver.close
          driver.closed?
          driver.write("hi")
          driver.read
        end
      end
    end
  end
end
