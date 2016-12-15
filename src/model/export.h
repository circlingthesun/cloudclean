#ifndef MODEL_EXPORT_H
    #define MODEL_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef model_EXPORTS
            #define MODEL_API Q_DECL_EXPORT
        #else
            #define MODEL_API Q_DECL_IMPORT
        #endif
    #else
        #define MODEL_API __attribute__ ((visibility ("default")))
    #endif
#endif
