/*


    g++ -o givescommand givescommand.cc


*/
#include <stdlib.h>
#include <iostream>



int main(int argc, char *argv[]){

//     system("./main&");  // with & its dificult to close
//     system("./main -p 6666 -r 2 -y 1");


    system("./main | ./main -p 6666 -r 2 -y 1 | ./main -p 6667 -r 3 -y 2 | ./main -p 6668 -r 4 -y 3 | ./main -p 6669 -r 5 -y 4 | ./main -p 6670 -r 6 -y 5 | ./main -p 6671 -r 7 -y 6 | ./main -p 6672 -r 8 -y 7");


    return 1;
} 
