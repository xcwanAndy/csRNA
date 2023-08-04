#include <stdio.h>
#include <unistd.h>

int main() {
    printf("This is an infinite loop ...\n");
    while (1) {
        sleep(1);
    }
    return 0;
}
