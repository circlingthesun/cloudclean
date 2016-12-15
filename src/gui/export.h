#ifndef GUI_EXPORT_H
    #define GUI_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef gui_EXPORTS
            #define GUI_API Q_DECL_EXPORT
        #else
            #define GUI_API Q_DECL_IMPORT
        #endif
    #else
        #define GUI_API __attribute__ ((visibility ("default")))
    #endif
#endif
