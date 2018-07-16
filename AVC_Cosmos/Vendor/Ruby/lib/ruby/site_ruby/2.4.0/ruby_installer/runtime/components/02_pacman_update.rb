module RubyInstaller
module Runtime # Rewrite from C:/projects/rubyinstaller2-hbuor/lib/ruby_installer/build/components/02_pacman_update.rb
module Components
class PacmanUpdate < Base
  def self.depends
    %w[msys2]
  end

  def description
    "MSYS2 system update"
  end

  def execute(args)
    msys.with_msys_apps_enabled do
      # Update the package database and core system packages
      puts "#{description} part 1  ..."
      res = run_verbose("pacman", "-Syu", *pacman_args)
      puts "#{description} #{res ? green("succeeded") : red("failed")}"
      raise "pacman failed" unless res

      # Update the rest
      puts "#{description} part 2 ..."
      res = run_verbose("pacman", "-Su", *pacman_args)
      puts "#{description} #{res ? green("succeeded") : red("failed")}"
      raise "pacman failed" unless res
    end
  end
end
end
end
end
