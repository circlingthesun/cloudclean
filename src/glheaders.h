#ifndef GLHEADERS_H_
#define GLHEADERS_H_
//#define QT_NO_OPENGL_ES_2
#ifdef _WIN32
#include <GL/glew.h>
#else
#define GL3_PROTOTYPES
#include <gl3.h>
#endif
#include <GL/glu.h>
#include <QDebug>
#include <QString>

#ifdef CC_GL_DEBUG_
    static GLenum gl_err = 0;
    #define CE() gl_err = glGetError();\
    if (gl_err != GL_NO_ERROR) {\
        const char* err_str = reinterpret_cast<const char *>(gluErrorString(gl_err));\
        QString errString(err_str);\
        qDebug() << "GL Error:" << errString << "on line" << __LINE__ << "of" << __FILE__;\
    }
#else
    #define CE()
#endif  // CC_GL_DEBUG_

#define RC(CODE) if(CODE == -1){\
    qDebug() << "Function call failed on line" << __LINE__ << "of" << __FILE__;}

#define ERR_MSG(err) QString(QString(err) + QString(" @ Line ") + QString::number(__LINE__) +  QString(" of ") +  QString(__FILE__))

class CheckSucc{
 public:
    CheckSucc(QString msg){
        succ = true;
        msg_ = msg;
    }

    CheckSucc operator = (bool rhs){
        succ = rhs;
        if(!rhs)
            qDebug() << msg_;
        return *this;
    }

    CheckSucc operator = (int rhs){
        if(rhs == -1){
            succ = false;
            qDebug() << msg_;
        }
        return *this;
    }

    bool operator == (bool rhs){
        return this->succ == rhs;
    }


 private:
    bool succ;
    QString msg_;
};

#define IF_FAIL(msg) CheckSucc(QString(QString(msg) + QString(" @ Line ") + QString::number(__LINE__) +  QString(" of ") +  QString(__FILE__)))


//
// Open CL helpers
//


// Helper function to get error string
// *********************************************************************
inline const char* oclErrorString(int error)
{
    static const char* errorString[] = {
        "CL_SUCCESS",
        "CL_DEVICE_NOT_FOUND",
        "CL_DEVICE_NOT_AVAILABLE",
        "CL_COMPILER_NOT_AVAILABLE",
        "CL_MEM_OBJECT_ALLOCATION_FAILURE",
        "CL_OUT_OF_RESOURCES",
        "CL_OUT_OF_HOST_MEMORY",
        "CL_PROFILING_INFO_NOT_AVAILABLE",
        "CL_MEM_COPY_OVERLAP",
        "CL_IMAGE_FORMAT_MISMATCH",
        "CL_IMAGE_FORMAT_NOT_SUPPORTED",
        "CL_BUILD_PROGRAM_FAILURE",
        "CL_MAP_FAILURE",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "CL_INVALID_VALUE",
        "CL_INVALID_DEVICE_TYPE",
        "CL_INVALID_PLATFORM",
        "CL_INVALID_DEVICE",
        "CL_INVALID_CONTEXT",
        "CL_INVALID_QUEUE_PROPERTIES",
        "CL_INVALID_COMMAND_QUEUE",
        "CL_INVALID_HOST_PTR",
        "CL_INVALID_MEM_OBJECT",
        "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
        "CL_INVALID_IMAGE_SIZE",
        "CL_INVALID_SAMPLER",
        "CL_INVALID_BINARY",
        "CL_INVALID_BUILD_OPTIONS",
        "CL_INVALID_PROGRAM",
        "CL_INVALID_PROGRAM_EXECUTABLE",
        "CL_INVALID_KERNEL_NAME",
        "CL_INVALID_KERNEL_DEFINITION",
        "CL_INVALID_KERNEL",
        "CL_INVALID_ARG_INDEX",
        "CL_INVALID_ARG_VALUE",
        "CL_INVALID_ARG_SIZE",
        "CL_INVALID_KERNEL_ARGS",
        "CL_INVALID_WORK_DIMENSION",
        "CL_INVALID_WORK_GROUP_SIZE",
        "CL_INVALID_WORK_ITEM_SIZE",
        "CL_INVALID_GLOBAL_OFFSET",
        "CL_INVALID_EVENT_WAIT_LIST",
        "CL_INVALID_EVENT",
        "CL_INVALID_OPERATION",
        "CL_INVALID_GL_OBJECT",
        "CL_INVALID_BUFFER_SIZE",
        "CL_INVALID_MIP_LEVEL",
        "CL_INVALID_GLOBAL_WORK_SIZE",
    };

    const int errorCount = sizeof(errorString) / sizeof(errorString[0]);

    const int index = -error;

    return (index >= 0 && index < errorCount) ? errorString[index] : "";

}

#endif  // GLHEADERS_H_
