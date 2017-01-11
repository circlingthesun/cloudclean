#ifndef PLUGINS_MARKOV_EXPORT_H
    #define PLUGINS_MARKOV_EXPORT_H
    #include <QtGlobal>
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef markov_EXPORTS
            #define MARKOV_API Q_DECL_EXPORT
        #else
            #define MARKOV_API Q_DECL_IMPORT
        #endif
    #else
        #define MARKOV_API __attribute__ ((visibility ("default")))
    #endif
#endif  // PLUGINS_MARKOV_EXPORT_H
