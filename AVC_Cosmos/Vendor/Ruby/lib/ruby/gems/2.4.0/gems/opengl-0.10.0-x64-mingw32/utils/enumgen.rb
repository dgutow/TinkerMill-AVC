#
# Copyright (C) 2007 Jan Dvorak <jan.dvorak@kraxnet.cz>
#
# This program is distributed under the terms of the MIT license.
# See the included MIT-LICENSE file for the terms of this license.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# enumgen.rb - generates GL/GLU enumerators code based on enum/enumext/gluenum .spec files

require 'open-uri'

def parse_enum_spec(infilename)
  puts "Reading #{infilename} ..."
  enum_list = {}
  f = open(infilename,"r")
  f.each do |line|
    case line
    when /^#|:|^$/ # skip comment, directive or empty line
      next
    when /^\tuse/ # enum alias for organization purpose, we don't need that
      next
    when /^\t|^ / # enum
      # remove trailing comment (if any) and split
      name,equals,value,*rest = line.split("#")[0].split(" ")
      # true/false are special constants
      unless (name=="TRUE" or name=="FALSE")
        enum_list[name] = value
      end
    else
      puts "Unexpected line: #{line}"
    end
  end
  f.close
  enum_list
end

def write_enums(enum_list, task)
  # .h
  puts "Writing #{task[:h]}"
  File.open(task[:h], "w") do |f|
    f << "/* This file was genereated on #{Time.now}" << "\n"
    task[:sources].each do |source|  f << "   source: #{source}" << "\n" end
    f << "*/" << "\n\n"
    enum_list.each do |name,value|
      gl_name = task[:prefix] + name
      f << "#ifndef #{gl_name}" << "\n"
      f << "#define #{gl_name} #{value}" << "\n"
      f << "#endif" << "\n"
    end
  end

  # .c
  puts "Writing #{task[:c]}"
  File.open(task[:c], "w") do |f|
    f << "/* This file was genereated on #{Time.now}" << "\n"
    task[:sources].each do |source|  f << "   source: #{source}" << "\n" end
    f << "*/" << "\n\n"
    f << '#include "common.h"' << "\n"
    f << "void #{task[:prefix].downcase}init_enums(VALUE module)" << "\n"
    f << "{" << "\n"

    # true/false are special constants
    f << "\trb_define_const(module, \"#{task[:prefix]}TRUE\", Qtrue);" << "\n"
    f << "\trb_define_const(module, \"#{task[:prefix]}FALSE\", Qfalse);" << "\n"
    f << "\n"

    enum_list.each do |name,value|
      gl_name = task[:prefix] + name
      f << "\trb_define_const(module, \"#{gl_name}\", INT2NUM(#{gl_name}));" << "\n"
    end

    f << "}" << "\n"
  end
end


# main
begin
  gl_enums = {:c => "../ext/opengl/gl-enums.c",:h => "../ext/opengl/gl-enums.h",
    :sources => ["https://www.opengl.org/registry/oldspecs/enum.spec",
    "https://www.opengl.org/registry/oldspecs/enumext.spec"],
    :prefix => "GL_"
  }

  task_list = [gl_enums]

  # main loop
  task_list.each do |task|
    enum_list = {}
    # gather enums from each source file/url
    task[:sources].each do |fname|
      enum_list.update(parse_enum_spec(fname))
    end
    # write to files
    write_enums(enum_list.sort,task)
  end
  puts "All OK"
rescue => e
  puts e
  puts e.backtrace
end
