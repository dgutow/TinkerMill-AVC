# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

require 'cosmos'
require 'cosmos/interfaces'
require 'cosmos/tools/cmd_tlm_server/interface_thread'

module Cosmos

  class PacketTarget

    class PacketServerInterface < TcpipServerInterface
      def initialize(port)
        @port = port
        super(port, port, 5.0, nil, 'LENGTH', 0, 32, 4, 1, 'BIG_ENDIAN', 4)
      end

      def pre_write_packet(packet)
        [packet.length].pack('N') << packet.buffer(false)
      end
    end

    class PacketInterfaceThread < InterfaceThread

      protected

      def handle_packet(packet)
        identified_packet = System.commands.identify(packet.buffer, [@port.to_s])
        if identified_packet
          Logger.info "Received command: #{identified_packet.target_name} #{identified_packet.packet_name}"
        else
          Logger.info "Received UNKNOWN command"
        end
      end
    end

    class PacketTelemetryThread
      attr_reader :thread

      def initialize(interface, port, packet, rate)
        @interface = interface
        @port = port.to_s
        @packet = packet.to_s
        @rate = rate.to_i
        @sleeper = Sleeper.new
      end

      def start
        packet = System.telemetry.packet(@port, @packet)
        packet.write("CCSDSAPID", packet.get_item("CCSDSAPID").id_value)
        packet.write("PKTID", packet.get_item("PKTID").id_value)
        @thread = Thread.new do
          @stop_thread = false
          begin
            while true
              @rate.times do
                (1..4).each do |item|
                  (1..10).each do |index|
                    packet.write("TEMP#{item}_#{index}", Random.rand(100))
                  end
                end
                packet.write('STRING', "The time is now: #{Time.now.formatted}")
                @interface.write(packet)
                break if @sleeper.sleep(1.0 / @rate)
              end
            end
          rescue Exception => err
            Logger.error "PacketTelemetryThread unexpectedly died\n#{err.formatted}"
          end
        end
      end

      def stop
        Cosmos.kill_thread(self, @thread)
      end

      def graceful_kill
        @sleeper.cancel
      end
    end

    def initialize(arguments)
      @port = arguments[0]
      @packet = arguments[1]
      @rate = arguments[2]
      # Create interface to receive commands and send telemetry
      @interface = PacketServerInterface.new(@port)
      @interface_thread = nil
      @telemetry_thread = nil
    end

    def start
      @interface_thread = PacketInterfaceThread.new(@interface)
      @interface_thread.start
      @telemetry_thread = PacketTelemetryThread.new(@interface, @port, @packet, @rate)
      @telemetry_thread.start
    end

    def stop
      @telemetry_thread.stop if @telemetry_thread
      @interface_thread.stop if @interface_thread
    end

    def self.run
      Logger.level = Logger::INFO
      target = self.new(ARGV)
      begin
        target.start
        while true
          sleep 1
        end
      rescue SystemExit, Interrupt
        target.stop
      end
    end

  end # class PacketTarget

end # module Cosmos
