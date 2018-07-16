# -*- encoding: utf-8 -*-
# stub: opengl 0.10.0 x64-mingw32 lib

Gem::Specification.new do |s|
  s.name = "opengl".freeze
  s.version = "0.10.0"
  s.platform = "x64-mingw32".freeze

  s.required_rubygems_version = Gem::Requirement.new(">= 0".freeze) if s.respond_to? :required_rubygems_version=
  s.require_paths = ["lib".freeze]
  s.authors = ["Eric Hodel".freeze, "Lars Kanis".freeze, "Bla\u017E Hrastnik".freeze, "Alain Hoang".freeze, "Jan Dvorak".freeze, "Minh Thu Vo".freeze, "James Adam".freeze]
  s.date = "2017-06-24"
  s.description = "An OpenGL wrapper for Ruby. opengl contains bindings for OpenGL.\n\nBe sure to check out\n{GLU}[https://github.com/larskanis/glu] and\n{GLUT}[https://github.com/larskanis/glut]\ngems.".freeze
  s.email = ["drbrain@segment7.net".freeze, "lars@greiz-reinsdorf.de".freeze, "blaz@mxxn.io".freeze, "".freeze, "".freeze, "".freeze, "".freeze]
  s.homepage = "https://github.com/larskanis/opengl".freeze
  s.licenses = ["MIT".freeze]
  s.rdoc_options = ["--main".freeze, "README.rdoc".freeze]
  s.required_ruby_version = Gem::Requirement.new(["< 2.5".freeze, ">= 2.0".freeze])
  s.rubygems_version = "2.6.13".freeze
  s.summary = "An OpenGL wrapper for Ruby".freeze

  s.installed_by_version = "2.6.13" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_development_dependency(%q<bundler>.freeze, ["~> 1.5"])
      s.add_development_dependency(%q<rake>.freeze, [">= 0"])
      s.add_development_dependency(%q<minitest>.freeze, ["~> 5.3"])
      s.add_development_dependency(%q<rake-compiler>.freeze, ["~> 1.0"])
      s.add_development_dependency(%q<rake-compiler-dock>.freeze, ["~> 0.6.0"])
      s.add_development_dependency(%q<glu>.freeze, ["~> 8.1"])
      s.add_development_dependency(%q<glut>.freeze, ["~> 8.1"])
    else
      s.add_dependency(%q<bundler>.freeze, ["~> 1.5"])
      s.add_dependency(%q<rake>.freeze, [">= 0"])
      s.add_dependency(%q<minitest>.freeze, ["~> 5.3"])
      s.add_dependency(%q<rake-compiler>.freeze, ["~> 1.0"])
      s.add_dependency(%q<rake-compiler-dock>.freeze, ["~> 0.6.0"])
      s.add_dependency(%q<glu>.freeze, ["~> 8.1"])
      s.add_dependency(%q<glut>.freeze, ["~> 8.1"])
    end
  else
    s.add_dependency(%q<bundler>.freeze, ["~> 1.5"])
    s.add_dependency(%q<rake>.freeze, [">= 0"])
    s.add_dependency(%q<minitest>.freeze, ["~> 5.3"])
    s.add_dependency(%q<rake-compiler>.freeze, ["~> 1.0"])
    s.add_dependency(%q<rake-compiler-dock>.freeze, ["~> 0.6.0"])
    s.add_dependency(%q<glu>.freeze, ["~> 8.1"])
    s.add_dependency(%q<glut>.freeze, ["~> 8.1"])
  end
end
