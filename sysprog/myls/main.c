#include "myls.h"

int main(int argc, char **argv)
{
    int ch;
    options_t *opt;
    const char *optstr = "adlRSt";
    char **args = NULL; /* stored to arguments of argv reffering to directories */
    int n_args = 0;
    opt = (options_t *)calloc(1, sizeof(options_t));
    if (opt == NULL)
        err_quit("calloc failed");
    while ( (ch = getopt(argc, argv, optstr)) != -1) {
        switch (ch) {
            case 'a':
                opt->flag_a = 1; break;
            case 'd':
                opt->flag_d = 1; break;
            case 'l':
                opt->flag_l = 1; break;
            case 'R':
                opt->flag_R = 1; break;
            case 'S':
                opt->flag_S = 1; opt->flag_t = 0; break;
            case 't':
                opt->flag_t = 1; opt->flag_S = 0; break;
            case '?':
                err_msg("unknown option");
        }
    }
    if (optind < argc) {
        args = (char **)calloc(argc - optind, sizeof(char *));
        while (optind < argc)
            args[n_args++] = argv[optind++];
    }
    myls(opt, args, n_args);
    free(args);
    free(opt);
    return 0;
}
