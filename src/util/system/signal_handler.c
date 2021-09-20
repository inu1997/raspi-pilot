#include <stdlib.h>
#include <signal.h>
#include <util/logger.h>

void signal_sigint_handler(int sig);

void signal_sigpipe_handler(int sig);

void (*_on_sigint)() = NULL;
void (*_on_sigpipe)() = NULL;

//-----
int signal_handler_init() {
    LOG("Initiating Signal handler.\n");

    signal(SIGPIPE, signal_sigpipe_handler);
    
    signal(SIGINT, signal_sigint_handler);
    
    LOG("Done.\n");
    return 0;
}

void signal_set_on_sigint(void (*func)()) {
    _on_sigint = func;
}

void signal_set_on_sigpipe(void (*func)()) {
    _on_sigpipe = func;
}

//-----

void signal_sigint_handler(int sig) {
    LOG("CTRL+C detected! Shutting down.\n");
    if(_on_sigint != NULL) {
        _on_sigint();
    }
    exit(0);
}

void signal_sigpipe_handler(int sig) {
    if (_on_sigpipe != NULL) {
        _on_sigpipe();
    }
    // Do nothing.
}