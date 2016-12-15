#ifndef COMMANDS_EXPORT_H
    #define COMMANDS_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef commands_EXPORTS
            #define COMMAND_API Q_DECL_EXPORT
        #else
            #define COMMAND_API Q_DECL_IMPORT
        #endif
    #else
        #define COMMAND_API __attribute__ ((visibility ("default")))
    #endif
#endif
