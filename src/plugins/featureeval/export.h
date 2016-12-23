#ifndef PLUGINS_FEATURE_EVAL_EXPORT_H
    #define PLUGINS_FEATURE_EVAL_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef feature_eval_EXPORTS
            #define FE_API Q_DECL_EXPORT
        #else
            #define FE_API Q_DECL_IMPORT
        #endif
    #else
        #define FE_API __attribute__ ((visibility ("default")))
    #endif
#endif  // PLUGINS_FEATURE_EVAL_EXPORT_H
