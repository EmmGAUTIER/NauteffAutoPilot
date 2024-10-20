#include <stdio.h>

int interpret (const char* YYCURSOR)
/*!re2c
    re2c:define:YYCTYPE = "char";
    re2c:define:YYFILL = "goto eof;";
    re2c:define:YYLIMIT = YYCURSOR + strlen(YYCURSOR);
    WS = [ \t];
    AP = "AP";
    
    
    AP_Heading   = {AP} " Heading" [ \t]+ [0-9]+;
    AP_Wind      = "AP Wind" [ \t]+ [0-9]+;
    AP_GPS       = "AP GPS";
    AP_Plus      = "AP" [ \t]* "+" [0-9]+;
    AP_Minus     = "AP" [ \t]* "-" [0-9]+;
    AP_StbBy     = "AP StbBy";
    AP_Auto      = "AP Auto";
    SET_Magnetic = "SET magnetic deflection" [ \t]+ [0-9]+ "." [0-9]+;

    * { /* Erreur de syntaxe */ }
    AP_Heading { /* Code C pour traiter AP Heading */ }
    AP_Wind    { /* Code C pour traiter AP Wind */ }
    AP_GPS     { printf ("Mode GPS\n"); }
    AP_Plus    { /* Code C pour traiter AP + */ }
    AP_Minus   { /* Code C pour traiter AP - */ }
    AP_StbBy   { /* Code C pour traiter AP StbBy */ }
    AP_Auto    { /* Code C pour traiter AP Auto */ }
    SET_Magnetic { /* Code C pour traiter SET magnetic deflection */ }
*/

int main (int argc, const char* argv)
{
    interpret("AP GPS");
}
