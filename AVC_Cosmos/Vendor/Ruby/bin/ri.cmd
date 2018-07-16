:""||{ ""=> %q<-*- ruby -*-
@"%~dp0ruby" -x "%~f0" %*
@exit /b %ERRORLEVEL%
};{ #
bindir="${0%/*}" #
exec "$bindir/ruby" -x "$0" "$@" #
>, #
} #
#!/mingw64/bin/ruby

begin
  gem 'rdoc'
rescue NameError => e # --disable-gems
  raise unless e.name == :gem
rescue Gem::LoadError
end

require 'rdoc/ri/driver'

RDoc::RI::Driver.run ARGV
