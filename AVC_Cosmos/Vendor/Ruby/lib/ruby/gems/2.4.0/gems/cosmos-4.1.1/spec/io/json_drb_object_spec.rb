# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'spec_helper'
require 'cosmos/io/json_drb_object'

module Cosmos

  describe JsonDRbObject do
    describe "initialize" do
      it "rescues bad hosts" do
        expect { JsonDRbObject.new("blah", 7777) }.to raise_error("Invalid hostname: blah")
      end
    end

    describe "method_missing" do
      it "calls the method on the remote object" do
        class JsonDRbObjectServer
          def my_method(param)
            param * 2
          end
        end

        json = JsonDRb.new
        json.start_service('127.0.0.1', 7777, JsonDRbObjectServer.new)
        obj = JsonDRbObject.new("localhost", 7777)
        expect(obj.my_method(10)).to eql 20
        obj.disconnect
        json.stop_service
        sleep(0.1)
      end

      it "raises an exception if the remote connection can't be made" do
        obj = JsonDRbObject.new("localhost", 7777)
        expect { obj.my_method(10) }.to raise_error(DRb::DRbConnError)
        obj.disconnect
        sleep(0.1)
      end

      it "retries the request and then raise an exception" do
        allow_any_instance_of(HTTPClient).to receive(:post) { raise "Error" }
        obj = JsonDRbObject.new("localhost", 7777)
        expect { obj.my_method(10) }.to raise_error(DRb::DRbConnError)
        obj.disconnect
        sleep(0.1)
      end

      it "raises an exception if the remote method returns an error" do
        json = JsonDRb.new
        json.start_service('127.0.0.1', 7777, self)
        obj = JsonDRbObject.new("localhost", 7777)
        expect { obj.no_such_method() }.to raise_error(NoMethodError)
        expect { obj.method_missing(:send) }.to raise_error(/Cannot call unauthorized methods/)
        obj.disconnect
        json.stop_service
        sleep(0.1)
      end

      it "handles the remote not returning a response" do
        class JsonDRbObjectServer
          def my_method(param)
            param * 2
          end
        end

        json = JsonDRb.new
        json.start_service('127.0.0.1', 7777, JsonDRbObjectServer.new)
        obj = JsonDRbObject.new("localhost", 7777)
        allow_any_instance_of(JsonDrbRack).to receive(:call) { sleep(5.0); nil }
        expect { obj.my_method(10) }.to raise_error(DRb::DRbConnError)
        obj.disconnect
        json.stop_service
        sleep(0.1)
      end

      it "handles the remote going away and coming back" do
        class JsonDRbObjectServer
          def my_method(param)
            param * 2
          end
        end

        json = JsonDRb.new
        json.start_service('127.0.0.1', 7777, JsonDRbObjectServer.new)
        obj = JsonDRbObject.new("localhost", 7777)
        expect(obj.my_method(10)).to eql 20
        json.stop_service
        json = JsonDRb.new
        json.start_service('127.0.0.1', 7777, JsonDRbObjectServer.new)
        expect(obj.my_method(10)).to eql 20
        obj.disconnect
        json.stop_service
        sleep(0.1)
      end

    end
  end
end

