/*
 * Copyright (C) 2007 Jan Dvorak <jan.dvorak@kraxnet.cz>
 *
 * This program is distributed under the terms of the MIT license.
 * See the included MIT-LICENSE file for the terms of this license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "common.h"

GL_FUNC_LOAD_2(BlendEquationSeparate,GLvoid, GLenum,GLenum, "2.0")
GL_FUNC_LOAD_4(StencilOpSeparate,GLvoid, GLenum,GLenum,GLenum,GLenum, "2.0")
GL_FUNC_LOAD_2(AttachShader,GLvoid, GLuint,GLuint, "2.0")
GL_FUNC_LOAD_1(CompileShader,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(DeleteProgram,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(DeleteShader,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_2(DetachShader,GLvoid, GLuint,GLuint, "2.0")
GL_FUNC_LOAD_1(DisableVertexAttribArray,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(EnableVertexAttribArray,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(LinkProgram,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(UseProgram,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_1(ValidateProgram,GLvoid, GLuint, "2.0")
GL_FUNC_LOAD_4(StencilFuncSeparate,GLvoid, GLenum,GLenum,GLint,GLuint, "2.0")
GL_FUNC_LOAD_2(StencilMaskSeparate,GLvoid, GLenum,GLuint, "2.0")
GL_FUNC_LOAD_0(CreateProgram,GLuint, "2.0")
GL_FUNC_LOAD_1(CreateShader,GLuint, GLenum, "2.0")
GL_FUNC_LOAD_1(IsProgram,GLboolean, GLuint, "2.0")
GL_FUNC_LOAD_1(IsShader,GLboolean, GLuint, "2.0")
GL_FUNC_LOAD_2(Uniform1f,GLvoid, GLint,GLfloat, "2.0")
GL_FUNC_LOAD_2(Uniform1i,GLvoid, GLint,GLint, "2.0")
GL_FUNC_LOAD_3(Uniform2f,GLvoid, GLint,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_3(Uniform2i,GLvoid, GLint,GLint,GLint, "2.0")
GL_FUNC_LOAD_4(Uniform3f,GLvoid, GLint,GLfloat,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_4(Uniform3i,GLvoid, GLint,GLint,GLint,GLint, "2.0")
GL_FUNC_LOAD_5(Uniform4f,GLvoid, GLint,GLfloat,GLfloat,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_5(Uniform4i,GLvoid, GLint,GLint,GLint,GLint,GLint, "2.0")
GL_FUNC_LOAD_2(VertexAttrib1d,GLvoid, GLuint,GLdouble, "2.0")
GL_FUNC_LOAD_2(VertexAttrib1f,GLvoid, GLuint,GLfloat, "2.0")
GL_FUNC_LOAD_2(VertexAttrib1s,GLvoid, GLuint,GLshort, "2.0")
GL_FUNC_LOAD_3(VertexAttrib2d,GLvoid, GLuint,GLdouble,GLdouble, "2.0")
GL_FUNC_LOAD_3(VertexAttrib2f,GLvoid, GLuint,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_3(VertexAttrib2s,GLvoid, GLuint,GLshort,GLshort, "2.0")
GL_FUNC_LOAD_4(VertexAttrib3d,GLvoid, GLuint,GLdouble,GLdouble,GLdouble, "2.0")
GL_FUNC_LOAD_4(VertexAttrib3f,GLvoid, GLuint,GLfloat,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_4(VertexAttrib3s,GLvoid, GLuint,GLshort,GLshort,GLshort, "2.0")
GL_FUNC_LOAD_5(VertexAttrib4d,GLvoid, GLuint,GLdouble,GLdouble,GLdouble,GLdouble, "2.0")
GL_FUNC_LOAD_5(VertexAttrib4f,GLvoid, GLuint,GLfloat,GLfloat,GLfloat,GLfloat, "2.0")
GL_FUNC_LOAD_5(VertexAttrib4s,GLvoid, GLuint,GLshort,GLshort,GLshort,GLshort, "2.0")
GL_FUNC_LOAD_5(VertexAttrib4Nub,GLvoid, GLuint,GLubyte,GLubyte,GLubyte,GLubyte, "2.0")

static VALUE
gl_DrawBuffers(obj,arg1)
VALUE obj,arg1;
{
  GLsizei size;
  GLenum *buffers;
  DECL_GL_FUNC_PTR(GLvoid,glDrawBuffers,(GLsizei,GLenum *));
  LOAD_GL_FUNC(glDrawBuffers, "2.0");
  Check_Type(arg1,T_ARRAY);
  size = (GLsizei)RARRAY_LENINT(arg1);
  buffers = ALLOC_N(GLenum,size);
  ary2cuint(arg1,buffers,size);
  fptr_glDrawBuffers(size,buffers);
  xfree(buffers);
  CHECK_GLERROR_FROM("glDrawBuffers");
  return Qnil;
}

static VALUE
gl_BindAttribLocation(obj,arg1,arg2,arg3)
VALUE obj,arg1,arg2,arg3;
{
  GLuint program;
  GLuint index;
  DECL_GL_FUNC_PTR(GLvoid,glBindAttribLocation,(GLuint,GLuint,GLchar *));
  LOAD_GL_FUNC(glBindAttribLocation, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  index = (GLuint)NUM2UINT(arg2);
  Check_Type(arg3, T_STRING);
  fptr_glBindAttribLocation(program,index,RSTRING_PTR(arg3));
  CHECK_GLERROR_FROM("glBindAttribLocation");
  return Qnil;
}

static VALUE
gl_GetProgramiv(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLenum pname;
  GLint params = 0;
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetProgramiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  pname = (GLenum)NUM2INT(arg2);
  fptr_glGetProgramiv(program,pname,&params);
  CHECK_GLERROR_FROM("glGetProgramiv");
  return cond_GLBOOL2RUBY(pname,params);
}

static VALUE
gl_GetActiveAttrib(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLuint index;
  GLsizei max_size = 0;
  GLsizei written = 0;
  GLint attrib_size = 0;
  GLenum attrib_type = 0;
  VALUE buffer;
  VALUE retary;
  DECL_GL_FUNC_PTR(GLvoid,glGetActiveAttrib,(GLuint,GLuint,GLsizei,GLsizei *,GLint *,GLenum *,GLchar *));
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetActiveAttrib, "2.0");
  LOAD_GL_FUNC(glGetProgramiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  index = (GLuint)NUM2UINT(arg2);
  fptr_glGetProgramiv(program,GL_ACTIVE_ATTRIBUTE_MAX_LENGTH,&max_size);
  CHECK_GLERROR_FROM("glGetProgramiv");
  /* Can't determine maximum attribute name length, so presume it */
  if (max_size==0) max_size = 256;
  buffer = allocate_buffer_with_string(max_size-1);

  fptr_glGetActiveAttrib(program,index,max_size,&written,&attrib_size,&attrib_type,RSTRING_PTR(buffer));

  rb_str_set_len(buffer, written);
  retary = rb_ary_new2(3);
  rb_ary_push(retary, INT2NUM(attrib_size));
  rb_ary_push(retary, INT2NUM(attrib_type));
  rb_ary_push(retary, buffer);
  CHECK_GLERROR_FROM("glGetActiveAttrib");
  return retary;
}

static VALUE
gl_GetActiveUniform(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLuint index;
  GLsizei max_size = 0;
  GLsizei written = 0;
  GLint uniform_size = 0;
  GLenum uniform_type = 0;
  VALUE buffer;
  VALUE retary;
  DECL_GL_FUNC_PTR(GLvoid,glGetActiveUniform,(GLuint,GLuint,GLsizei,GLsizei*,GLint*,GLenum*,GLchar*));
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetActiveUniform, "2.0");
  LOAD_GL_FUNC(glGetProgramiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  index = (GLuint)NUM2UINT(arg2);
  fptr_glGetProgramiv(program,GL_ACTIVE_UNIFORM_MAX_LENGTH,&max_size);
  CHECK_GLERROR_FROM("glGetProgramiv");
  /* Can't determine maximum uniform name length, so presume it */
  if (max_size==0) max_size = 256;
  buffer = allocate_buffer_with_string(max_size-1);

  fptr_glGetActiveUniform(program,index,max_size,&written,&uniform_size,&uniform_type,RSTRING_PTR(buffer));

  rb_str_set_len(buffer, written);
  retary = rb_ary_new2(3);
  rb_ary_push(retary, INT2NUM(uniform_size));
  rb_ary_push(retary, INT2NUM(uniform_type));
  rb_ary_push(retary, buffer);
  CHECK_GLERROR_FROM("glGetActiveUniform");
  return retary;
}

static VALUE
gl_GetAttachedShaders(obj,arg1)
VALUE obj,arg1;
{
  GLuint program;
  GLint shaders_num = 0;
  GLuint *shaders;
  GLsizei count = 0;
  DECL_GL_FUNC_PTR(GLvoid,glGetAttachedShaders,(GLuint,GLsizei,GLsizei *,GLuint *));
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetAttachedShaders, "2.0");
  LOAD_GL_FUNC(glGetProgramiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  fptr_glGetProgramiv(program,GL_ATTACHED_SHADERS,&shaders_num);
  CHECK_GLERROR_FROM("glGetProgramiv");
  if (shaders_num<=0)
    return Qnil;
  shaders = ALLOC_N(GLuint,shaders_num);
  fptr_glGetAttachedShaders(program,shaders_num,&count,shaders);
  RET_ARRAY_OR_SINGLE_FREE("glGetAttachedShaders", count, RETCONV_GLuint,
      shaders);
}

static VALUE
gl_GetAttribLocation(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLint ret;
  DECL_GL_FUNC_PTR(GLint,glGetAttribLocation,(GLuint, GLchar *));
  LOAD_GL_FUNC(glGetAttribLocation, "2.0");
  program=(GLuint)NUM2UINT(arg1);
  Check_Type(arg2,T_STRING);
  ret = fptr_glGetAttribLocation(program,RSTRING_PTR(arg2));
  CHECK_GLERROR_FROM("glGetAttribLocation");
  return INT2NUM(ret);
}

static VALUE
gl_GetProgramInfoLog(obj,arg1)
VALUE obj,arg1;
{
  GLuint program;
  GLint max_size = 0;
  GLsizei ret_length = 0;
  VALUE buffer;
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramInfoLog,(GLuint,GLsizei,GLsizei *,GLchar *));
  DECL_GL_FUNC_PTR(GLvoid,glGetProgramiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetProgramInfoLog, "2.0");
  LOAD_GL_FUNC(glGetProgramiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  fptr_glGetProgramiv(program,GL_INFO_LOG_LENGTH,&max_size);
  CHECK_GLERROR_FROM("glGetProgramiv");
  if (max_size<=0)
    return rb_str_new2("");
  buffer = allocate_buffer_with_string(max_size);
  fptr_glGetProgramInfoLog(program,max_size,&ret_length,RSTRING_PTR(buffer));
  rb_str_set_len(buffer, ret_length);
  CHECK_GLERROR_FROM("glGetProgramInfoLog");
  return buffer;
}

static VALUE
gl_GetShaderiv(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLenum pname;
  GLint params = 0;
  DECL_GL_FUNC_PTR(GLvoid,glGetShaderiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetShaderiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  pname = (GLenum)NUM2INT(arg2);
  fptr_glGetShaderiv(program,pname,&params);
  CHECK_GLERROR_FROM("glGetShaderiv");
  return cond_GLBOOL2RUBY(pname,params);
}

static VALUE
gl_GetShaderInfoLog(obj,arg1)
VALUE obj,arg1;
{
  GLuint program;
  GLint max_size = 0;
  GLsizei ret_length = 0;
  VALUE ret_buffer;
  GLchar *buffer;
  DECL_GL_FUNC_PTR(GLvoid,glGetShaderInfoLog,(GLuint,GLsizei,GLsizei *,GLchar *));
  DECL_GL_FUNC_PTR(GLvoid,glGetShaderiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetShaderInfoLog, "2.0");
  LOAD_GL_FUNC(glGetShaderiv, "2.0");
  program = (GLuint)NUM2UINT(arg1);
  fptr_glGetShaderiv(program,GL_INFO_LOG_LENGTH,&max_size);
  CHECK_GLERROR_FROM("glGetShaderiv");
  if (max_size<=0)
    return rb_str_new2("");
  buffer = ALLOC_N(GLchar,max_size+1);
  memset(buffer,0,sizeof(GLchar) * (max_size+1));
  fptr_glGetShaderInfoLog(program,max_size,&ret_length,buffer);
  ret_buffer = rb_str_new(buffer, ret_length);
  xfree(buffer);
  CHECK_GLERROR_FROM("glGetShaderInfoLog");
  return ret_buffer;
}

static VALUE
gl_GetShaderSource(obj,arg1)
VALUE obj,arg1;
{
  GLuint shader;
  GLint max_size = 0;
  GLsizei ret_length = 0;
  VALUE buffer;
  DECL_GL_FUNC_PTR(GLvoid,glGetShaderSource,(GLuint,GLsizei,GLsizei *,GLchar *));
  DECL_GL_FUNC_PTR(GLvoid,glGetShaderiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetShaderSource, "2.0");
  LOAD_GL_FUNC(glGetShaderiv, "2.0");
  shader = (GLuint)NUM2UINT(arg1);
  fptr_glGetShaderiv(shader,GL_SHADER_SOURCE_LENGTH,&max_size);
  CHECK_GLERROR_FROM("glGetShaderiv");
  if (max_size==0)
    rb_raise(rb_eTypeError, "Can't determine maximum shader source length");
  buffer = allocate_buffer_with_string(max_size-1);
  fptr_glGetShaderSource(shader,max_size,&ret_length,RSTRING_PTR(buffer));
  CHECK_GLERROR_FROM("glGetShaderSource");
  return buffer;
}

static VALUE
gl_GetUniformLocation(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint program;
  GLint ret;
  DECL_GL_FUNC_PTR(GLint,glGetUniformLocation,(GLuint,const GLchar*));
  LOAD_GL_FUNC(glGetUniformLocation, "2.0");
  program=(GLuint)NUM2UINT(arg1);
  Check_Type(arg2,T_STRING);
  ret = fptr_glGetUniformLocation(program, RSTRING_PTR(arg2));

  CHECK_GLERROR_FROM("glGetUniformLocation");
  return INT2NUM(ret);
}

#define GETUNIFORM_FUNC(_name_,_type_) \
static VALUE \
gl_##_name_(obj,arg1,arg2) \
VALUE obj,arg1,arg2; \
{ \
  GLuint program; \
  GLint location; \
  _type_ params[16]; \
  GLint unused = 0; \
  GLenum uniform_type = 0; \
  GLint uniform_size = 0; \
\
  DECL_GL_FUNC_PTR(GLvoid,gl##_name_,(GLuint,GLint,_type_ *)); \
  DECL_GL_FUNC_PTR(GLvoid,glGetActiveUniform,(GLuint,GLuint,GLsizei,GLsizei*,GLint*,GLenum*,GLchar*)); \
  LOAD_GL_FUNC(gl##_name_, "2.0"); \
  LOAD_GL_FUNC(glGetActiveUniform, "2.0"); \
  program = (GLuint)NUM2UINT(arg1); \
  location = (GLint)NUM2INT(arg2); \
\
  fptr_glGetActiveUniform(program,location,0,NULL,&unused,&uniform_type,NULL); \
  CHECK_GLERROR_FROM("glGetActiveUniform"); \
  if (uniform_type==0) \
    rb_raise(rb_eTypeError, "Can't determine the uniform's type"); \
\
  uniform_size = get_uniform_size(uniform_type); \
\
  memset(params,0,16*sizeof(_type_)); \
  fptr_gl##_name_(program,location,params); \
  RET_ARRAY_OR_SINGLE("gl" #_name_, uniform_size, RETCONV_##_type_, params); \
}

GETUNIFORM_FUNC(GetUniformfv,GLfloat)
GETUNIFORM_FUNC(GetUniformiv,GLint)
#undef GETUNIFORM_FUNC

#define GETVERTEXATTRIB_FUNC(_name_,_type_) \
static VALUE \
gl_##_name_(obj,arg1,arg2) \
VALUE obj,arg1,arg2; \
{ \
  GLuint index; \
  GLenum pname; \
  _type_ params[4] = {0,0,0,0}; \
  GLint size; \
  DECL_GL_FUNC_PTR(GLvoid,gl##_name_,(GLuint,GLenum,_type_ *)); \
  LOAD_GL_FUNC(gl##_name_, "2.0"); \
  index = (GLuint)NUM2UINT(arg1); \
  pname = (GLenum)NUM2INT(arg2); \
  if (pname==GL_CURRENT_VERTEX_ATTRIB) \
    size = 4; \
  else \
    size = 1; \
  fptr_gl##_name_(index,pname,params); \
  RET_ARRAY_OR_SINGLE("gl" #_name_, size, RETCONV_##_type_, params); \
}

GETVERTEXATTRIB_FUNC(GetVertexAttribdv,GLdouble)
GETVERTEXATTRIB_FUNC(GetVertexAttribfv,GLfloat)
//GETVERTEXATTRIB_FUNC(GetVertexAttribiv,GLint,cond_GLBOOL2RUBY)
#undef GETVERTEXATTRIB_FUNC

static VALUE
gl_GetVertexAttribiv(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint index;
  GLenum pname;
  GLint params[4] = {0,0,0,0};
  GLint size;
  DECL_GL_FUNC_PTR(GLvoid,glGetVertexAttribiv,(GLuint,GLenum,GLint *));
  LOAD_GL_FUNC(glGetVertexAttribiv, "2.0");
  index = (GLuint)NUM2UINT(arg1);
  pname = (GLenum)NUM2INT(arg2);
  if (pname==GL_CURRENT_VERTEX_ATTRIB)
    size = 4;
  else
    size = 1;
  fptr_glGetVertexAttribiv(index,pname,params);
  RET_ARRAY_OR_SINGLE_BOOL("glGetVertexAttribiv", size, cond_GLBOOL2RUBY,
      pname, params);
}

static VALUE
gl_GetVertexAttribPointerv(obj,arg1)
VALUE obj,arg1;
{
  GLuint index;
  DECL_GL_FUNC_PTR(GLvoid,glGetVertexAttribPointerv,(GLuint,GLenum,GLvoid **));
  LOAD_GL_FUNC(glGetVertexAttribPointerv, "2.0");
  index =(GLuint) NUM2INT(arg1);
  if (index>_MAX_VERTEX_ATTRIBS)
    rb_raise(rb_eArgError, "Index too large, maximum allowed value '%i'",_MAX_VERTEX_ATTRIBS);

  return GET_GLIMPL_VARIABLE(VertexAttrib_ptr)[index];
}

static VALUE
gl_ShaderSource(obj,arg1,arg2)
VALUE obj,arg1,arg2;
{
  GLuint shader;
  GLint length;
  GLchar *str;
  DECL_GL_FUNC_PTR(GLvoid,glShaderSource,(GLuint,GLsizei,GLchar**,GLint *));
  LOAD_GL_FUNC(glShaderSource, "2.0");
  shader = (GLuint)NUM2UINT(arg1);
  Check_Type(arg2,T_STRING);
  str = RSTRING_PTR(arg2);
  length = (GLint)RSTRING_LENINT(arg2);
  fptr_glShaderSource(shader,1,&str,&length);
  CHECK_GLERROR_FROM("glShaderSource");
  return Qnil;
}

#define UNIFORM_FUNC_V(_name_,_type_,_conv_,_size_) \
static VALUE \
gl_##_name_(obj,arg1,arg2) \
VALUE obj,arg1,arg2; \
{ \
  GLint location; \
  GLsizei count; \
  _type_ *value; \
  DECL_GL_FUNC_PTR(GLvoid,gl##_name_,(GLint,GLsizei,_type_ *)); \
  LOAD_GL_FUNC(gl##_name_,"2.0"); \
  Check_Type(arg2,T_ARRAY); \
  count = (GLsizei)RARRAY_LENINT(arg2); \
  if (count<=0 || (count % _size_) != 0) \
    rb_raise(rb_eArgError, "Parameter array size must be multiplication of %i",_size_); \
  location = (GLint)NUM2INT(arg1); \
  value = ALLOC_N(_type_,count); \
  _conv_(arg2,value,count); \
  fptr_gl##_name_(location,count / _size_,value); \
  xfree(value); \
  CHECK_GLERROR_FROM("gl" #_name_); \
  return Qnil; \
}

UNIFORM_FUNC_V(Uniform1fv,GLfloat,ary2cflt,1)
UNIFORM_FUNC_V(Uniform1iv,GLint,ary2cint,1)
UNIFORM_FUNC_V(Uniform2fv,GLfloat,ary2cflt,2)
UNIFORM_FUNC_V(Uniform2iv,GLint,ary2cint,2)
UNIFORM_FUNC_V(Uniform3fv,GLfloat,ary2cflt,3)
UNIFORM_FUNC_V(Uniform3iv,GLint,ary2cint,3)
UNIFORM_FUNC_V(Uniform4fv,GLfloat,ary2cflt,4)
UNIFORM_FUNC_V(Uniform4iv,GLint,ary2cint,4)
#undef UNIFORM_FUNC_V

#define UNIFORMMATRIX_FUNC(_name_,_size_) \
static VALUE \
gl_##_name_(obj,arg1,arg2,arg3) \
VALUE obj,arg1,arg2,arg3; \
{ \
  GLint location; \
  GLsizei count; \
  GLboolean transpose; \
  GLfloat *value; \
  DECL_GL_FUNC_PTR(GLvoid,gl##_name_,(GLint,GLsizei,GLboolean,GLfloat *)); \
  LOAD_GL_FUNC(gl##_name_, "2.0"); \
  location = (GLint)NUM2INT(arg1); \
  count = (GLsizei)RARRAY_LENINT(rb_funcall(rb_Array(arg3), rb_intern("flatten"), 0)); \
  transpose = (GLboolean)RUBYBOOL2GL(arg2); \
  value = ALLOC_N(GLfloat, count); \
  ary2cmatfloatcount(arg3,value,_size_,_size_); \
  fptr_gl##_name_(location,count / (_size_*_size_),transpose,value); \
  xfree(value); \
  CHECK_GLERROR_FROM("gl" #_name_); \
  return Qnil; \
}

UNIFORMMATRIX_FUNC(UniformMatrix2fv,2)
UNIFORMMATRIX_FUNC(UniformMatrix3fv,3)
UNIFORMMATRIX_FUNC(UniformMatrix4fv,4)
#undef UNIFORMMATRIX_FUNC

#define VERTEXATTRIB_FUNC_V(_name_,_type_,_conv_,_size_) \
static VALUE \
gl_##_name_(obj,arg1,arg2) \
VALUE obj,arg1,arg2; \
{ \
  GLuint index; \
  _type_ v[_size_]; \
  DECL_GL_FUNC_PTR(GLvoid,gl##_name_,(GLuint,_type_ *)); \
  LOAD_GL_FUNC(gl##_name_, "2.0"); \
  index = (GLuint)NUM2UINT(arg1); \
  _conv_(arg2,v,_size_); \
  fptr_gl##_name_(index,v); \
  CHECK_GLERROR_FROM("gl" #_name_); \
  return Qnil; \
}

VERTEXATTRIB_FUNC_V(VertexAttrib4Nbv,GLbyte,ary2cbyte,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4Niv,GLint,ary2cint,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4Nsv,GLshort,ary2cshort,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4Nubv,GLubyte,ary2cubyte,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4Nuiv,GLuint,ary2cuint,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4Nusv,GLushort,ary2cushort,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4uiv,GLuint,ary2cuint,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4ubv,GLubyte,ary2cubyte,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4usv,GLushort,ary2cushort,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4bv,GLbyte,ary2cbyte,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4iv,GLint,ary2cint,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4dv,GLdouble,ary2cdbl,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4fv,GLfloat,ary2cflt,4)
VERTEXATTRIB_FUNC_V(VertexAttrib4sv,GLshort,ary2cshort,4)
VERTEXATTRIB_FUNC_V(VertexAttrib3dv,GLdouble,ary2cdbl,3)
VERTEXATTRIB_FUNC_V(VertexAttrib3fv,GLfloat,ary2cflt,3)
VERTEXATTRIB_FUNC_V(VertexAttrib3sv,GLshort,ary2cshort,3)
VERTEXATTRIB_FUNC_V(VertexAttrib2dv,GLdouble,ary2cdbl,2)
VERTEXATTRIB_FUNC_V(VertexAttrib2fv,GLfloat,ary2cflt,2)
VERTEXATTRIB_FUNC_V(VertexAttrib2sv,GLshort,ary2cshort,2)
VERTEXATTRIB_FUNC_V(VertexAttrib1dv,GLdouble,ary2cdbl,1)
VERTEXATTRIB_FUNC_V(VertexAttrib1fv,GLfloat,ary2cflt,1)
VERTEXATTRIB_FUNC_V(VertexAttrib1sv,GLshort,ary2cshort,1)
#undef VERTEXATTRIB_FUNC_V

static VALUE
gl_VertexAttribPointer(obj,arg1,arg2,arg3,arg4,arg5,arg6)
VALUE obj,arg1,arg2,arg3,arg4,arg5,arg6;
{
  GLuint index;
  GLuint size;
  GLenum type;
  GLboolean normalized;
  GLsizei stride;
  DECL_GL_FUNC_PTR(GLvoid,glVertexAttribPointer,(GLuint,GLint,GLenum,GLboolean,GLsizei,GLvoid *));
  LOAD_GL_FUNC(glVertexAttribPointer, "2.0");
  index = (GLuint)NUM2UINT(arg1);
  size = (GLuint)NUM2UINT(arg2);
  type = (GLenum)NUM2INT(arg3);
  normalized = (GLboolean)RUBYBOOL2GL(arg4);
  stride = (GLsizei)NUM2UINT(arg5);
  if (index>_MAX_VERTEX_ATTRIBS)
    rb_raise(rb_eArgError, "Index too large, maximum allowed value '%i'",_MAX_VERTEX_ATTRIBS);

  if (CHECK_BUFFER_BINDING(GL_ARRAY_BUFFER_BINDING)) {
    GET_GLIMPL_VARIABLE(VertexAttrib_ptr)[index] = arg6;
    fptr_glVertexAttribPointer(index,size,type,normalized,stride,(GLvoid *)NUM2SIZET(arg6));
  } else {
    VALUE data;
    data = pack_array_or_pass_string(type,arg6);
    rb_str_freeze(data);
    GET_GLIMPL_VARIABLE(VertexAttrib_ptr)[index] = data;
    fptr_glVertexAttribPointer(index,size,type,normalized,stride,(GLvoid *)RSTRING_PTR(data));
  }
  CHECK_GLERROR_FROM("glVertexAttribPointer");
  return Qnil;
}

void gl_init_functions_2_0(VALUE klass)
{
  rb_define_method(klass, "glBlendEquationSeparate", gl_BlendEquationSeparate, 2);
  rb_define_method(klass, "glDrawBuffers", gl_DrawBuffers, 1);
  rb_define_method(klass, "glStencilOpSeparate", gl_StencilOpSeparate, 4);
  rb_define_method(klass, "glStencilFuncSeparate", gl_StencilFuncSeparate, 4);
  rb_define_method(klass, "glStencilMaskSeparate", gl_StencilMaskSeparate, 2);
  rb_define_method(klass, "glAttachShader", gl_AttachShader, 2);
  rb_define_method(klass, "glBindAttribLocation", gl_BindAttribLocation, 3);
  rb_define_method(klass, "glCompileShader", gl_CompileShader, 1);
  rb_define_method(klass, "glCreateProgram", gl_CreateProgram, 0);
  rb_define_method(klass, "glCreateShader", gl_CreateShader, 1);
  rb_define_method(klass, "glDeleteProgram", gl_DeleteProgram, 1);
  rb_define_method(klass, "glDeleteShader", gl_DeleteShader, 1);
  rb_define_method(klass, "glDetachShader", gl_DetachShader, 2);
  rb_define_method(klass, "glDisableVertexAttribArray", gl_DisableVertexAttribArray, 1);
  rb_define_method(klass, "glEnableVertexAttribArray", gl_EnableVertexAttribArray, 1);
  rb_define_method(klass, "glGetActiveAttrib", gl_GetActiveAttrib, 2);
  rb_define_method(klass, "glGetActiveUniform", gl_GetActiveUniform, 2);
  rb_define_method(klass, "glGetAttachedShaders", gl_GetAttachedShaders, 1);
  rb_define_method(klass, "glGetAttribLocation", gl_GetAttribLocation, 2);
  rb_define_method(klass, "glGetProgramiv", gl_GetProgramiv, 2);
  rb_define_method(klass, "glGetProgramInfoLog", gl_GetProgramInfoLog, 1);
  rb_define_method(klass, "glGetShaderiv", gl_GetShaderiv, 2);
  rb_define_method(klass, "glGetShaderInfoLog", gl_GetShaderInfoLog, 1);
  rb_define_method(klass, "glGetShaderSource", gl_GetShaderSource, 1);
  rb_define_method(klass, "glGetUniformLocation", gl_GetUniformLocation, 2);
  rb_define_method(klass, "glGetUniformfv", gl_GetUniformfv, 2);
  rb_define_method(klass, "glGetUniformiv", gl_GetUniformiv, 2);
  rb_define_method(klass, "glGetVertexAttribdv", gl_GetVertexAttribdv, 2);
  rb_define_method(klass, "glGetVertexAttribfv", gl_GetVertexAttribfv, 2);
  rb_define_method(klass, "glGetVertexAttribiv", gl_GetVertexAttribiv, 2);
  rb_define_method(klass, "glGetVertexAttribPointerv", gl_GetVertexAttribPointerv, 1);
  rb_define_method(klass, "glIsProgram", gl_IsProgram, 1);
  rb_define_method(klass, "glIsShader", gl_IsShader, 1);
  rb_define_method(klass, "glLinkProgram", gl_LinkProgram, 1);
  rb_define_method(klass, "glShaderSource", gl_ShaderSource, 2);
  rb_define_method(klass, "glUseProgram", gl_UseProgram, 1);
  rb_define_method(klass, "glUniform1f", gl_Uniform1f, 2);
  rb_define_method(klass, "glUniform2f", gl_Uniform2f, 3);
  rb_define_method(klass, "glUniform3f", gl_Uniform3f, 4);
  rb_define_method(klass, "glUniform4f", gl_Uniform4f, 5);
  rb_define_method(klass, "glUniform1i", gl_Uniform1i, 2);
  rb_define_method(klass, "glUniform2i", gl_Uniform2i, 3);
  rb_define_method(klass, "glUniform3i", gl_Uniform3i, 4);
  rb_define_method(klass, "glUniform4i", gl_Uniform4i, 5);
  rb_define_method(klass, "glUniform1fv", gl_Uniform1fv, 2);
  rb_define_method(klass, "glUniform2fv", gl_Uniform2fv, 2);
  rb_define_method(klass, "glUniform3fv", gl_Uniform3fv, 2);
  rb_define_method(klass, "glUniform4fv", gl_Uniform4fv, 2);
  rb_define_method(klass, "glUniform1iv", gl_Uniform1iv, 2);
  rb_define_method(klass, "glUniform2iv", gl_Uniform2iv, 2);
  rb_define_method(klass, "glUniform3iv", gl_Uniform3iv, 2);
  rb_define_method(klass, "glUniform4iv", gl_Uniform4iv, 2);
  rb_define_method(klass, "glUniformMatrix2fv", gl_UniformMatrix2fv, 3);
  rb_define_method(klass, "glUniformMatrix3fv", gl_UniformMatrix3fv, 3);
  rb_define_method(klass, "glUniformMatrix4fv", gl_UniformMatrix4fv, 3);
  rb_define_method(klass, "glValidateProgram", gl_ValidateProgram, 1);
  rb_define_method(klass, "glVertexAttrib1d", gl_VertexAttrib1d, 2);
  rb_define_method(klass, "glVertexAttrib1f", gl_VertexAttrib1f, 2);
  rb_define_method(klass, "glVertexAttrib1s", gl_VertexAttrib1s, 2);
  rb_define_method(klass, "glVertexAttrib2d", gl_VertexAttrib2d, 3);
  rb_define_method(klass, "glVertexAttrib2f", gl_VertexAttrib2f, 3);
  rb_define_method(klass, "glVertexAttrib2s", gl_VertexAttrib2s, 3);
  rb_define_method(klass, "glVertexAttrib3d", gl_VertexAttrib3d, 4);
  rb_define_method(klass, "glVertexAttrib3f", gl_VertexAttrib3f, 4);
  rb_define_method(klass, "glVertexAttrib3s", gl_VertexAttrib3s, 4);
  rb_define_method(klass, "glVertexAttrib4Nbv", gl_VertexAttrib4Nbv, 2);
  rb_define_method(klass, "glVertexAttrib4Niv", gl_VertexAttrib4Niv, 2);
  rb_define_method(klass, "glVertexAttrib4Nsv", gl_VertexAttrib4Nsv, 2);
  rb_define_method(klass, "glVertexAttrib4Nub", gl_VertexAttrib4Nub, 5);
  rb_define_method(klass, "glVertexAttrib4Nubv", gl_VertexAttrib4Nubv, 2);
  rb_define_method(klass, "glVertexAttrib4Nuiv", gl_VertexAttrib4Nuiv, 2);
  rb_define_method(klass, "glVertexAttrib4Nusv", gl_VertexAttrib4Nusv, 2);
  rb_define_method(klass, "glVertexAttrib4bv", gl_VertexAttrib4bv, 2);
  rb_define_method(klass, "glVertexAttrib4d", gl_VertexAttrib4d, 5);
  rb_define_method(klass, "glVertexAttrib4f", gl_VertexAttrib4f, 5);
  rb_define_method(klass, "glVertexAttrib4iv", gl_VertexAttrib4iv, 2);
  rb_define_method(klass, "glVertexAttrib4s", gl_VertexAttrib4s, 5);
  rb_define_method(klass, "glVertexAttrib4ubv", gl_VertexAttrib4ubv, 2);
  rb_define_method(klass, "glVertexAttrib4uiv", gl_VertexAttrib4uiv, 2);
  rb_define_method(klass, "glVertexAttrib4usv", gl_VertexAttrib4usv, 2);
  rb_define_method(klass, "glVertexAttrib1dv", gl_VertexAttrib1dv, 2);
  rb_define_method(klass, "glVertexAttrib1fv", gl_VertexAttrib1fv, 2);
  rb_define_method(klass, "glVertexAttrib1sv", gl_VertexAttrib1sv, 2);
  rb_define_method(klass, "glVertexAttrib2dv", gl_VertexAttrib2dv, 2);
  rb_define_method(klass, "glVertexAttrib2fv", gl_VertexAttrib2fv, 2);
  rb_define_method(klass, "glVertexAttrib2sv", gl_VertexAttrib2sv, 2);
  rb_define_method(klass, "glVertexAttrib3dv", gl_VertexAttrib3dv, 2);
  rb_define_method(klass, "glVertexAttrib3fv", gl_VertexAttrib3fv, 2);
  rb_define_method(klass, "glVertexAttrib3sv", gl_VertexAttrib3sv, 2);
  rb_define_method(klass, "glVertexAttrib4dv", gl_VertexAttrib4dv, 2);
  rb_define_method(klass, "glVertexAttrib4fv", gl_VertexAttrib4fv, 2);
  rb_define_method(klass, "glVertexAttrib4sv", gl_VertexAttrib4sv, 2);
  rb_define_method(klass, "glVertexAttribPointer", gl_VertexAttribPointer, 6);
}
