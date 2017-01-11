#ifndef PLUGINS_ACCURACY_EXPORT_H
    #define PLUGINS_ACCURACY_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef accuracy_EXPORTS
            #define ACCURACY_API Q_DECL_EXPORT
        #else
            #define ACCURACY_API Q_DECL_IMPORT
        #endif
    #else
        #define ACCURACY_API __attribute__ ((visibility ("default")))
    #endif
#endif  // PLUGINS_ACCURACY_EXPORT_H
