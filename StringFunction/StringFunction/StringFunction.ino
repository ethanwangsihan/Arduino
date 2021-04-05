void setup () {
   Serial.begin (9600);
   Serial.print ("According to isdigit:\r");
   Serial.print (isdigit( '8' ) ? "8 is a": "8 is not a");
   Serial.print (" digit\r" );
   Serial.print (isdigit( '8' ) ?"# is a": "# is not a") ;
   Serial.print (" digit\r");
   Serial.print ("\rAccording to isalpha:\r" );
   Serial.print (isalpha('A' ) ?"A is a": "A is not a");
   Serial.print (" letter\r");
   Serial.print (isalpha('A' ) ?"b is a": "b is not a");
   Serial.print (" letter\r");
   Serial.print (isalpha('A') ?"& is a": "& is not a");
   Serial.print (" letter\r");
   Serial.print (isalpha( 'A' ) ?"4 is a":"4 is not a");
   Serial.print (" letter\r");
   Serial.print ("\rAccording to isalnum:\r");
   Serial.print (isalnum( 'A' ) ?"A is a" : "A is not a" );

   Serial.print (" digit or a letter\r" );
   Serial.print (isalnum( '8' ) ?"8 is a" : "8 is not a" ) ;
   Serial.print (" digit or a letter\r");
   Serial.print (isalnum( '#' ) ?"# is a" : "# is not a" );
   Serial.print (" digit or a letter\r");
   Serial.print ("\rAccording to isxdigit:\r");
   Serial.print (isxdigit( 'F' ) ?"F is a" : "F is not a" );
   Serial.print (" hexadecimal digit\r" );
   Serial.print (isxdigit( 'J' ) ?"J is a" : "J is not a" ) ;
   Serial.print (" hexadecimal digit\r" );
   Serial.print (isxdigit( '7' ) ?"7 is a" : "7 is not a" ) ;

   Serial.print (" hexadecimal digit\r" );
   Serial.print (isxdigit( '$' ) ? "$ is a" : "$ is not a" );
   Serial.print (" hexadecimal digit\r" );
   Serial.print (isxdigit( 'f' ) ? "f is a" : "f is not a");
   ///////////////////////////////////////////


   Serial.print ("According to islower:\r") ;
   Serial.print (islower( 'p' ) ? "p is a" : "p is not a" );
   Serial.print ( " lowercase letter\r" );
   Serial.print ( islower( 'P') ? "P is a" : "P is not a") ;
   Serial.print ("lowercase letter\r");
   Serial.print (islower( '5' ) ? "5 is a" : "5 is not a" );
   Serial.print ( " lowercase letter\r" );
   Serial.print ( islower( '!' )? "! is a" : "! is not a") ;
   Serial.print ("lowercase letter\r");

   Serial.print ("\rAccording to isupper:\r") ;
   Serial.print (isupper ( 'D' ) ? "D is a" : "D is not an" );
   Serial.print ( " uppercase letter\r" );
   Serial.print ( isupper ( 'd' )? "d is a" : "d is not an") ;
   Serial.print ( " uppercase letter\r" );
   Serial.print (isupper ( '8' ) ? "8 is a" : "8 is not an" );
   Serial.print ( " uppercase letter\r" );
   Serial.print ( islower( '$' )? "$ is a" : "$ is not an") ;
   Serial.print ("uppercase letter\r ");

   ////////////////////////////////////////////
   Serial.print ( " According to isspace:\rNewline ") ;
   Serial.print (isspace( '\n' )? " is a" : " is not a" );
   Serial.print ( " whitespace character\rHorizontal tab") ;
   Serial.print (isspace( '\t' )? " is a" : " is not a" );
   Serial.print ( " whitespace character\n") ;
   Serial.print (isspace('%')? " % is a" : " % is not a" );
   
   Serial.print ( " \rAccording to iscntrl:\rNewline") ;
   Serial.print ( iscntrl( '\n' )?"is a" : " is not a" ) ;
   Serial.print (" control character\r");
   Serial.print (iscntrl( '$' ) ? " $ is a" : " $ is not a" );
   Serial.print (" control character\r");
   Serial.print ("\rAccording to ispunct:\r");
   Serial.print (ispunct(';' ) ?"; is a" : "; is not a" ) ;
   Serial.print (" punctuation character\r");
   Serial.print (ispunct('Y' ) ?"Y is a" : "Y is not a" ) ;
   Serial.print ("punctuation character\r");
   Serial.print (ispunct('#' ) ?"# is a" : "# is not a" ) ;
   Serial.print ("punctuation character\r");

   Serial.print ( "\r According to isprint:\r");
   Serial.print (isprint('$' ) ?"$ is a" : "$ is not a" );
   Serial.print (" printing character\rAlert ");
   Serial.print (isprint('\a' ) ?" is a" : " is not a" );
   Serial.print (" printing character\rSpace ");
   Serial.print (isprint(' ' ) ?" is a" : " is not a" );
   Serial.print (" printing character\r");
   
   Serial.print ("\r According to isgraph:\r");
   Serial.print (isgraph ('Q' ) ?"Q is a" : "Q is not a" );
   Serial.print ("printing character other than a space\rSpace ");
   Serial.print (isgraph (' ') ?" is a" : " is not a" );
   Serial.print ("printing character other than a space ");
}

void loop () {

}
