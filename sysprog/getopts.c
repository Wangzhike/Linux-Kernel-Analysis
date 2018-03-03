#include <stdio.h>
#include <unistd.h>
#include "/home/qiuyu/code/Linux-Kernel-Analysis/err_handlers/err_handlers.h"

int main(int argc, char **argv)
{
    int opt;
    const char *prompt;

    while ( (opt = getopt(argc, argv, ":adlRSt")) != -1) {
        switch (opt) {
            case 'a':
                prompt = "all"; break;
            case 'd':
                prompt = "directory not content"; break;
            case 'l':
                prompt = "long format output"; break;
            case 'R':
                prompt = "recursively"; break;
            case 'S':
                prompt = "sorted by file size"; break;
            case 't':
                prompt = "sorted by modified time"; break;
            case '?':
                prompt = "unknown option"; break;
            case ':':
                prompt = "miss argument"; break;
        }
        printf("%s\n", prompt);
    }
    return 0;
}
