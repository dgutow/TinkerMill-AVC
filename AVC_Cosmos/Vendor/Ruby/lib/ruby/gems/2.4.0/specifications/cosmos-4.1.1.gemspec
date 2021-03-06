# -*- encoding: utf-8 -*-
# stub: cosmos 4.1.1 ruby lib
# stub: ext/cosmos/ext/array/extconf.rb ext/cosmos/ext/buffered_file/extconf.rb ext/cosmos/ext/config_parser/extconf.rb ext/cosmos/ext/cosmos_io/extconf.rb ext/cosmos/ext/crc/extconf.rb ext/cosmos/ext/line_graph/extconf.rb ext/cosmos/ext/low_fragmentation_array/extconf.rb ext/cosmos/ext/packet/extconf.rb ext/cosmos/ext/platform/extconf.rb ext/cosmos/ext/polynomial_conversion/extconf.rb ext/cosmos/ext/string/extconf.rb ext/cosmos/ext/tabbed_plots_config/extconf.rb ext/cosmos/ext/telemetry/extconf.rb ext/mkrf_conf.rb

Gem::Specification.new do |s|
  s.name = "cosmos".freeze
  s.version = "4.1.1"

  s.required_rubygems_version = Gem::Requirement.new(">= 0".freeze) if s.respond_to? :required_rubygems_version=
  s.require_paths = ["lib".freeze]
  s.authors = ["Ryan Melton".freeze, "Jason Thomas".freeze]
  s.date = "2017-12-06"
  s.description = "    Ball Aerospace COSMOS provides all the functionality needed to send\n    commands to and receive data from one or more embedded systems\n    referred to as \"targets\". Out of the box functionality includes:\n    Telemetry Display, Telemetry Graphing, Operational and Test Scripting,\n    Command Sending, Logging, Log File Playback, Table Management, and more.\n".freeze
  s.email = ["rmelton@ball.com".freeze, "jmthomas@ball.com".freeze]
  s.executables = ["cosmos".freeze, "rubysloc".freeze, "cstol_converter".freeze, "xtce_converter".freeze]
  s.extensions = ["ext/cosmos/ext/array/extconf.rb".freeze, "ext/cosmos/ext/buffered_file/extconf.rb".freeze, "ext/cosmos/ext/config_parser/extconf.rb".freeze, "ext/cosmos/ext/cosmos_io/extconf.rb".freeze, "ext/cosmos/ext/crc/extconf.rb".freeze, "ext/cosmos/ext/line_graph/extconf.rb".freeze, "ext/cosmos/ext/low_fragmentation_array/extconf.rb".freeze, "ext/cosmos/ext/packet/extconf.rb".freeze, "ext/cosmos/ext/platform/extconf.rb".freeze, "ext/cosmos/ext/polynomial_conversion/extconf.rb".freeze, "ext/cosmos/ext/string/extconf.rb".freeze, "ext/cosmos/ext/tabbed_plots_config/extconf.rb".freeze, "ext/cosmos/ext/telemetry/extconf.rb".freeze, "ext/mkrf_conf.rb".freeze]
  s.files = ["bin/cosmos".freeze, "bin/cstol_converter".freeze, "bin/rubysloc".freeze, "bin/xtce_converter".freeze, "ext/cosmos/ext/array/extconf.rb".freeze, "ext/cosmos/ext/buffered_file/extconf.rb".freeze, "ext/cosmos/ext/config_parser/extconf.rb".freeze, "ext/cosmos/ext/cosmos_io/extconf.rb".freeze, "ext/cosmos/ext/crc/extconf.rb".freeze, "ext/cosmos/ext/line_graph/extconf.rb".freeze, "ext/cosmos/ext/low_fragmentation_array/extconf.rb".freeze, "ext/cosmos/ext/packet/extconf.rb".freeze, "ext/cosmos/ext/platform/extconf.rb".freeze, "ext/cosmos/ext/polynomial_conversion/extconf.rb".freeze, "ext/cosmos/ext/string/extconf.rb".freeze, "ext/cosmos/ext/tabbed_plots_config/extconf.rb".freeze, "ext/cosmos/ext/telemetry/extconf.rb".freeze, "ext/mkrf_conf.rb".freeze]
  s.homepage = "https://github.com/BallAerospace/COSMOS".freeze
  s.licenses = ["GPL-3.0".freeze]
  s.post_install_message = "Thanks for installing Ball Aerospace COSMOS!\nStart your first project with: cosmos demo demo\n".freeze
  s.required_ruby_version = Gem::Requirement.new("~> 2.2".freeze)
  s.rubygems_version = "2.6.13".freeze
  s.summary = "Ball Aerospace COSMOS".freeze

  s.installed_by_version = "2.6.13" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_runtime_dependency(%q<bundler>.freeze, ["~> 1.3"])
      s.add_runtime_dependency(%q<rdoc>.freeze, ["< 5", ">= 3"])
      s.add_runtime_dependency(%q<rake>.freeze, ["< 13", ">= 0"])
      s.add_runtime_dependency(%q<json>.freeze, ["< 1.9", ">= 1.5"])
      s.add_runtime_dependency(%q<pry>.freeze, ["< 0.11", ">= 0.9"])
      s.add_runtime_dependency(%q<pry-doc>.freeze, ["< 0.7", ">= 0.5"])
      s.add_runtime_dependency(%q<yard>.freeze, ["< 0.10", ">= 0.8"])
      s.add_runtime_dependency(%q<uuidtools>.freeze, ["~> 2.1.0"])
      s.add_runtime_dependency(%q<snmp>.freeze, ["~> 1.0"])
      s.add_runtime_dependency(%q<rubyzip>.freeze, ["~> 1.2.1"])
      s.add_runtime_dependency(%q<nokogiri>.freeze, ["~> 1.6"])
      s.add_runtime_dependency(%q<opengl>.freeze, ["~> 0.10"])
      s.add_runtime_dependency(%q<qtbindings>.freeze, [">= 4.8.6.2", "~> 4.8.6"])
      s.add_runtime_dependency(%q<puma>.freeze, ["~> 3.10.0"])
      s.add_runtime_dependency(%q<rack>.freeze, ["~> 2.0.3"])
      s.add_runtime_dependency(%q<httpclient>.freeze, ["~> 2.8.3"])
      s.add_development_dependency(%q<rspec>.freeze, ["~> 3.5.0"])
      s.add_development_dependency(%q<flog>.freeze, ["~> 4.0"])
      s.add_development_dependency(%q<flay>.freeze, ["~> 2.0"])
      s.add_development_dependency(%q<reek>.freeze, ["~> 1.0"])
      s.add_development_dependency(%q<roodi>.freeze, ["~> 4.0"])
      s.add_development_dependency(%q<guard>.freeze, ["~> 2.0"])
      s.add_development_dependency(%q<listen>.freeze, ["~> 2.0"])
      s.add_development_dependency(%q<guard-bundler>.freeze, ["~> 2.0"])
      s.add_development_dependency(%q<guard-rspec>.freeze, ["~> 4.0"])
      s.add_development_dependency(%q<simplecov>.freeze, ["~> 0.11"])
      s.add_development_dependency(%q<codecov>.freeze, ["~> 0.1"])
      s.add_development_dependency(%q<benchmark-ips>.freeze, ["~> 2.0"])
      s.add_development_dependency(%q<ruby-prof>.freeze, ["~> 0.15.0"])
    else
      s.add_dependency(%q<bundler>.freeze, ["~> 1.3"])
      s.add_dependency(%q<rdoc>.freeze, ["< 5", ">= 3"])
      s.add_dependency(%q<rake>.freeze, ["< 13", ">= 0"])
      s.add_dependency(%q<json>.freeze, ["< 1.9", ">= 1.5"])
      s.add_dependency(%q<pry>.freeze, ["< 0.11", ">= 0.9"])
      s.add_dependency(%q<pry-doc>.freeze, ["< 0.7", ">= 0.5"])
      s.add_dependency(%q<yard>.freeze, ["< 0.10", ">= 0.8"])
      s.add_dependency(%q<uuidtools>.freeze, ["~> 2.1.0"])
      s.add_dependency(%q<snmp>.freeze, ["~> 1.0"])
      s.add_dependency(%q<rubyzip>.freeze, ["~> 1.2.1"])
      s.add_dependency(%q<nokogiri>.freeze, ["~> 1.6"])
      s.add_dependency(%q<opengl>.freeze, ["~> 0.10"])
      s.add_dependency(%q<qtbindings>.freeze, [">= 4.8.6.2", "~> 4.8.6"])
      s.add_dependency(%q<puma>.freeze, ["~> 3.10.0"])
      s.add_dependency(%q<rack>.freeze, ["~> 2.0.3"])
      s.add_dependency(%q<httpclient>.freeze, ["~> 2.8.3"])
      s.add_dependency(%q<rspec>.freeze, ["~> 3.5.0"])
      s.add_dependency(%q<flog>.freeze, ["~> 4.0"])
      s.add_dependency(%q<flay>.freeze, ["~> 2.0"])
      s.add_dependency(%q<reek>.freeze, ["~> 1.0"])
      s.add_dependency(%q<roodi>.freeze, ["~> 4.0"])
      s.add_dependency(%q<guard>.freeze, ["~> 2.0"])
      s.add_dependency(%q<listen>.freeze, ["~> 2.0"])
      s.add_dependency(%q<guard-bundler>.freeze, ["~> 2.0"])
      s.add_dependency(%q<guard-rspec>.freeze, ["~> 4.0"])
      s.add_dependency(%q<simplecov>.freeze, ["~> 0.11"])
      s.add_dependency(%q<codecov>.freeze, ["~> 0.1"])
      s.add_dependency(%q<benchmark-ips>.freeze, ["~> 2.0"])
      s.add_dependency(%q<ruby-prof>.freeze, ["~> 0.15.0"])
    end
  else
    s.add_dependency(%q<bundler>.freeze, ["~> 1.3"])
    s.add_dependency(%q<rdoc>.freeze, ["< 5", ">= 3"])
    s.add_dependency(%q<rake>.freeze, ["< 13", ">= 0"])
    s.add_dependency(%q<json>.freeze, ["< 1.9", ">= 1.5"])
    s.add_dependency(%q<pry>.freeze, ["< 0.11", ">= 0.9"])
    s.add_dependency(%q<pry-doc>.freeze, ["< 0.7", ">= 0.5"])
    s.add_dependency(%q<yard>.freeze, ["< 0.10", ">= 0.8"])
    s.add_dependency(%q<uuidtools>.freeze, ["~> 2.1.0"])
    s.add_dependency(%q<snmp>.freeze, ["~> 1.0"])
    s.add_dependency(%q<rubyzip>.freeze, ["~> 1.2.1"])
    s.add_dependency(%q<nokogiri>.freeze, ["~> 1.6"])
    s.add_dependency(%q<opengl>.freeze, ["~> 0.10"])
    s.add_dependency(%q<qtbindings>.freeze, [">= 4.8.6.2", "~> 4.8.6"])
    s.add_dependency(%q<puma>.freeze, ["~> 3.10.0"])
    s.add_dependency(%q<rack>.freeze, ["~> 2.0.3"])
    s.add_dependency(%q<httpclient>.freeze, ["~> 2.8.3"])
    s.add_dependency(%q<rspec>.freeze, ["~> 3.5.0"])
    s.add_dependency(%q<flog>.freeze, ["~> 4.0"])
    s.add_dependency(%q<flay>.freeze, ["~> 2.0"])
    s.add_dependency(%q<reek>.freeze, ["~> 1.0"])
    s.add_dependency(%q<roodi>.freeze, ["~> 4.0"])
    s.add_dependency(%q<guard>.freeze, ["~> 2.0"])
    s.add_dependency(%q<listen>.freeze, ["~> 2.0"])
    s.add_dependency(%q<guard-bundler>.freeze, ["~> 2.0"])
    s.add_dependency(%q<guard-rspec>.freeze, ["~> 4.0"])
    s.add_dependency(%q<simplecov>.freeze, ["~> 0.11"])
    s.add_dependency(%q<codecov>.freeze, ["~> 0.1"])
    s.add_dependency(%q<benchmark-ips>.freeze, ["~> 2.0"])
    s.add_dependency(%q<ruby-prof>.freeze, ["~> 0.15.0"])
  end
end
