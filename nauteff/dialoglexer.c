/* Generated by re2c 3.0 on Tue Jul 30 00:09:53 2024 */
#line 1 "dialog.re"
#include <stdio.h>

int interpret (const char* YYCURSOR)

#line 8 "dialoglexer.c"
{
	char yych;
	if ((YYLIMIT - YYCURSOR) < 12) goto eof;(12);
	yych = *YYCURSOR;
	switch (yych) {
		case 'A': goto yy3;
		case 'S': goto yy4;
		default: goto yy1;
	}
yy1:
	++YYCURSOR;
yy2:
#line 20 "dialog.re"
	{ /* Erreur de syntaxe */ }
#line 23 "dialoglexer.c"
yy3:
	yych = *(YYMARKER = ++YYCURSOR);
	switch (yych) {
		case 'P': goto yy5;
		default: goto yy2;
	}
yy4:
	yych = *(YYMARKER = ++YYCURSOR);
	switch (yych) {
		case 'E': goto yy7;
		default: goto yy2;
	}
yy5:
	yych = *++YYCURSOR;
	switch (yych) {
		case '_': goto yy8;
		default: goto yy6;
	}
yy6:
	YYCURSOR = YYMARKER;
	goto yy2;
yy7:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'T': goto yy9;
		default: goto yy6;
	}
yy8:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'A': goto yy10;
		case 'G': goto yy11;
		case 'H': goto yy12;
		case 'M': goto yy13;
		case 'P': goto yy14;
		case 'S': goto yy15;
		case 'W': goto yy16;
		default: goto yy6;
	}
yy9:
	yych = *++YYCURSOR;
	switch (yych) {
		case '_': goto yy17;
		default: goto yy6;
	}
yy10:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'u': goto yy18;
		default: goto yy6;
	}
yy11:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'P': goto yy19;
		default: goto yy6;
	}
yy12:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'e': goto yy20;
		default: goto yy6;
	}
yy13:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'i': goto yy21;
		default: goto yy6;
	}
yy14:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'l': goto yy22;
		default: goto yy6;
	}
yy15:
	yych = *++YYCURSOR;
	switch (yych) {
		case 't': goto yy23;
		default: goto yy6;
	}
yy16:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'i': goto yy24;
		default: goto yy6;
	}
yy17:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'M': goto yy25;
		default: goto yy6;
	}
yy18:
	yych = *++YYCURSOR;
	switch (yych) {
		case 't': goto yy26;
		default: goto yy6;
	}
yy19:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'S': goto yy27;
		default: goto yy6;
	}
yy20:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'a': goto yy28;
		default: goto yy6;
	}
yy21:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'n': goto yy29;
		default: goto yy6;
	}
yy22:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'u': goto yy30;
		default: goto yy6;
	}
yy23:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'b': goto yy31;
		default: goto yy6;
	}
yy24:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'n': goto yy32;
		default: goto yy6;
	}
yy25:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'a': goto yy33;
		default: goto yy6;
	}
yy26:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'o': goto yy34;
		default: goto yy6;
	}
yy27:
	++YYCURSOR;
#line 23 "dialog.re"
	{ printf ("Mode GPS\n"); }
#line 175 "dialoglexer.c"
yy28:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'd': goto yy35;
		default: goto yy6;
	}
yy29:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'u': goto yy36;
		default: goto yy6;
	}
yy30:
	yych = *++YYCURSOR;
	switch (yych) {
		case 's': goto yy37;
		default: goto yy6;
	}
yy31:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'B': goto yy38;
		default: goto yy6;
	}
yy32:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'd': goto yy39;
		default: goto yy6;
	}
yy33:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'g': goto yy40;
		default: goto yy6;
	}
yy34:
	++YYCURSOR;
#line 27 "dialog.re"
	{ /* Code C pour traiter AP Auto */ }
#line 216 "dialoglexer.c"
yy35:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'i': goto yy41;
		default: goto yy6;
	}
yy36:
	yych = *++YYCURSOR;
	switch (yych) {
		case 's': goto yy42;
		default: goto yy6;
	}
yy37:
	++YYCURSOR;
#line 24 "dialog.re"
	{ /* Code C pour traiter AP + */ }
#line 233 "dialoglexer.c"
yy38:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'y': goto yy43;
		default: goto yy6;
	}
yy39:
	++YYCURSOR;
#line 22 "dialog.re"
	{ /* Code C pour traiter AP Wind */ }
#line 244 "dialoglexer.c"
yy40:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'n': goto yy44;
		default: goto yy6;
	}
yy41:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'n': goto yy45;
		default: goto yy6;
	}
yy42:
	++YYCURSOR;
#line 25 "dialog.re"
	{ /* Code C pour traiter AP - */ }
#line 261 "dialoglexer.c"
yy43:
	++YYCURSOR;
#line 26 "dialog.re"
	{ /* Code C pour traiter AP StbBy */ }
#line 266 "dialoglexer.c"
yy44:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'e': goto yy46;
		default: goto yy6;
	}
yy45:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'g': goto yy47;
		default: goto yy6;
	}
yy46:
	yych = *++YYCURSOR;
	switch (yych) {
		case 't': goto yy48;
		default: goto yy6;
	}
yy47:
	++YYCURSOR;
#line 21 "dialog.re"
	{ /* Code C pour traiter AP Heading */ }
#line 289 "dialoglexer.c"
yy48:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'i': goto yy49;
		default: goto yy6;
	}
yy49:
	yych = *++YYCURSOR;
	switch (yych) {
		case 'c': goto yy50;
		default: goto yy6;
	}
yy50:
	++YYCURSOR;
#line 28 "dialog.re"
	{ /* Code C pour traiter SET magnetic deflection */ }
#line 306 "dialoglexer.c"
}
#line 29 "dialog.re"


int main (int argc, const char* argv)
{
    interpret("AP GPS");
}
