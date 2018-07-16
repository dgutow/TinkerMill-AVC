# encoding: ascii-8bit

# Copyright 2014 Ball Aerospace & Technologies Corp.
# All Rights Reserved.
#
# This program is free software; you can modify and/or redistribute it
# under the terms of the GNU General Public License
# as published by the Free Software Foundation; version 3 with
# attribution addendums as found in the LICENSE.txt

# This file updates the Ruby SNMP gem for non-blocking sockets
# to work around potential process lockups.
#
# Known to work with SNMP Version 1.0.2

# Note requires SNMP Ruby Gem - gem install snmp
require 'snmp'

old_verbose = $VERBOSE; $VERBOSE = nil
module SNMP
  class UDPTransport
    def recv(max_bytes)
      # Implement blocking using recvfrom_nonblock to prevent potential
      # ruby thread deadlock
      begin
        data, host_info = @socket.recvfrom_nonblock(max_bytes)
        return data
      rescue Errno::EAGAIN, Errno::EWOULDBLOCK
        IO.fast_select([@socket])
        retry
      end
    end
  end

  class UDPServerTransport
    def recvfrom(max_bytes)
      # Implement blocking using recvfrom_nonblock to prevent potential
      # ruby thread deadlock
      begin
        data, host_info = @socket.recvfrom_nonblock(max_bytes)
      rescue Errno::EAGAIN, Errno::EWOULDBLOCK
        IO.fast_select([@socket])
        retry
      end
      flags, host_port, host_name, host_ip = host_info
      return data, host_ip, host_port
    end
  end
end
$VERBOSE = old_verbose
