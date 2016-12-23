#ifndef PLUGINS_PROJECT_EXPORT_H
    #define PLUGINS_PROJECT_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef project_EXPORTS
            #define PROJECT_API Q_DECL_EXPORT
        #else
            #define PROJECT_API Q_DECL_IMPORT
        #endif
    #else
        #define PROJECT_API __attribute__ ((visibility ("default")))
    #endif
#endif  // PLUGINS_PROJECT_EXPORT_H
