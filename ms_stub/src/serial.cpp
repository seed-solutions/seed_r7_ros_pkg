#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pty.h>
#include <string.h>
#include <unistd.h>
#include <pty.h>
#include <utmp.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <iostream>


int open_pseudo_serial(const char *dev_path, int *mfd, int *sfd) {
    int master, slave;
    char name[256];

    auto e = openpty(&master, &slave, &name[0], NULL, NULL);
    if (0 > e) {
        printf("openpty Error: %s\n", strerror(errno));
        return -1;
    }

    remove(dev_path);
    if (symlink(name, dev_path) != 0) {
        printf("symlink Error: %s\n", strerror(errno));
        close(slave);
        close(master);
        return -1;
    }

    if (fchmod(slave, S_IROTH | S_IWOTH) != 0){
        printf("fchmod Error: %s\n", strerror(errno));
        close(slave);
        close(master);
        return -1;
    }

    if (chmod(dev_path, 0777) != 0){
        printf("fchmod Error: %s\n", strerror(errno));
        close(slave);
        close(master);
        return -1;
    }

    *mfd = master;
    *sfd = slave;
    printf("%s created\n", dev_path);

    return 0;
}
