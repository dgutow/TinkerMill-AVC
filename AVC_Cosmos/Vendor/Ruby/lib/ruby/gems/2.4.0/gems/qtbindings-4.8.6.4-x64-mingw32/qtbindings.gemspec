require './lib/qtbindings_version'

spec = Gem::Specification.new do |s|
  s.authors = ['Ryan Melton', 'Jason Thomas', 'Richard Dale', 'Arno Rehn']
  s.email = 'kde-bindings@kde.org'
  s.rubyforge_project = 'qtbindings'
  s.summary = "Qt bindings for ruby"
  s.homepage = "http://github.com/ryanmelt/qtbindings"
  s.name = 'qtbindings'
  s.version = QTBINDINGS_VERSION
  s.requirements << 'none'
  s.require_path = 'lib'
  s.files = Dir['lib/**/*', 'bin/**/*', 'examples/**/*', 'ext/**/*', '*.txt', 'extconf.rb', '*.gemspec', 'Rakefile'].to_a
  s.extensions = ['extconf.rb']
  s.executables = ['smokeapi', 'smokedeptool', 'rbrcc', 'rbuic4', 'rbqtapi']
  s.description = 'qtbindings provides ruby bindings to QT4.x. It is derived from the kdebindings project.'
  s.licenses = ['LGPL-2.1']
end
