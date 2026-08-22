//#include "test_geom.h"

int test_geom();
int test_util();

int main (int argc, const char* argv[])
{
    int errnb = 0;

    errnb += test_geom();
    errnb += test_util();

    return errnb;
}
