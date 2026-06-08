#include <stdio.h>
#include <ctype.h>

int main() {
    printf("Value\tisalpha\tisdigit\tisalnum\tisxdigit\tisprint\tisgraph\tisspace\tisupper\tislower\tiscntrl\tispunct\n");
    for (int i = 0; i < 256; i++) {
        printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
               i,
               isalpha(i),
               isdigit(i),
               isalnum(i),
               isxdigit(i),
               isprint(i),
               isgraph(i),
               isspace(i),
               isupper(i),
               islower(i),
               iscntrl(i),
               ispunct(i));
    }
    return 0;
}