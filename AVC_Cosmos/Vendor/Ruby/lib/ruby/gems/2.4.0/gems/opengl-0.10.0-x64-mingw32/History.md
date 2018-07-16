=== 0.10.0 / 2017-06-25

* Add binaries for RubyInstaller up to 2.4 in Windows gems.
* Make RedBook examples compatible with Ruby 2.0+.
* Replace hoe by bundler and add some release automatisms.
* Use GL_PIXEL_PACK_BUFFER instead of _ARB version.
* Add OpenGL-3.0 vertex array functions.
* Ship gl.h and glext.h with the gem, which are taken from the Mesa sources.
* Update gl-enums to the latest version based on enum.spec.
* Move linking of the OpenGL library from build time to run time.
  This allowes loading of GL implementations that are not integrated into
  the OS (like OSMesa on Windows) or to load several implementations
  into the Ruby process at the same time.
* Remove OpenGL::VERSION in favour of OpenGL::BINDINGS_VERSION and GL::BINDINGS_VERSION.

=== 0.9.2 / 2015-01-05

* Fix compatibility with Ruby-2.2.0.
* Add ruby-2.2 to binary windows gems.

=== 0.9.1 / 2014-10-06

* Add ruby-2.1 to binary windows gems.

=== 0.9.0 / 2014-03-14

* Split glu and glut into seperate gems.
* Add OpenGL 3.0 support.
* Add x64-mingw platform for cross build and add ruby-2.0.0 to binary gems.
* Don't pollute the global namespace. Use GL namespace.

* Replace UINT2FIX with UINT2NUM since UINT2FIX is not defined in MRI ruby.
* Don't return the terminating null from C in GetProgramInfoLog.
* Remove the outdated website, use gh-pages now.
* Fixes in tests, resolved some upstream mesa bugs.
* glMaterial: Before converting using to_a, check if conversion is possible.
* Fix incorrect conversion of bool arguments to functions, they were getting
  converted from C instead of to C (`GL_TRUE/true` got converted to `41`).
* Fix: some parts of GL_EXT_gpu_shader4 were checking for GL_ARB_vertex_shader
support instead of GL_EXT_gpu_shader4.
* Fix ProgramVertexLimitNV checked for GL_NV_gpu_program4 support instead of
GL_EXT_geometry_shader4.
* Fix build with Ruby-2.0 on OS X Mavericks.

* Dropped support for 1.2 optional ARB_imaging subset. It is deprecated and
support for it was dropped from all major drivers. Continuing support for it
would lead to developers using deprecated coding practices.

* Fix several test cases.

=== 0.8.0 / 2013-02-03

* Drop support for Ruby 1.8.
* Wrap glut callbacks in GVL release/acquire for better thread support.
* Add OpenGL::Buffer for mapped buffers that allows writing.

=== 0.7.0 / 2013-02-03

* Better support of 64 bit systems.
* Replace mkrf with extconf.rb, hoe and rake-compiler.
* Switch to dlopen() from deprecated NSAddImage() and friends for OS X.
* glBegin, glPushMatrix, glEnable and glEnableClientState now accept a block.
* glEnable/glDisable and glEnableClientState/glDisableClientState now accept
  multiple arguments.
* Allow to_a-able objects for glColor*v, glRasterPos*v, glRect*v, glTexCoord*v,
  glMaterial, glNormal and glVertex.
* Fix unused param count in ARY2CMAT.
* Fix a lot of bugs in the test suite.
* Merge opengl C extensions into a single extension to avoid code duplication
  in extconf.rb and simplify rebuilding based on header changes.
* Add cross compilation tasks with download of freeglut for binary windows gems

=== 0.60.1 / 2009-02-16

* Bugfixes
* Proper support for ruby 1.9/1.9.1+
* Updated OpenGL enumerators in preparation for OpenGL 3.0

=== 0.60.0 / 2008-01-06

* Automatic error checking for GL/GLU calls, enabled by default (see
  doc/tutorial)
* Added support for many more OpenGL extensions
* Support for Ruby 1.9.0+ (requires mkrf 0.2.3)
* Ton of bugfixes.

* API Changes:
	* Boolean functions/parameters was changed to ruby true/false instead of
	  GL_TRUE / GL_FALSE, which remains for compatibility
	* glGet\* functions now returns +x+ instead of <code>[x]</code> when
	  returning only one value
	* Functions operating on packed strings (glTexture, glPointer etc.) now
	  also accepts ruby arrays directly
	* Matrix handling functions now also accepts instances of Matrix class, or
	  any class that can be converted to array
	* glUniform*v and glUniformmatrix*v now does not require 'count'
	  parameter, they will calculate it from length of passed array
	* glCallLists needs type specifier (previously was forced to GL_BYTE)
	* On ruby 1.9, glut keyboard callback returns char ("x") instead of integer
	  so using 'if key == ?x' works on both 1.8 and 1.9

