#include <iostream>
#include <math.h>
#include <vector>   
using namespace std;

 

int main () 
{
  
 
float x = 0;
  
 
float y = 0;
  
 
float angle = 0.0f;
  
 
 
// Space between the spirals
  int a = 2, b = 2;
  
  vector<vector<float>> vec( 50 , vector<float> (3));
 
for (int i = 0; i < 50; i++) {
 
angle = 0.20 * i;
     
 
x = (a + b * angle) * cos (angle);
      
y = (a + b * angle) * sin (angle);
      

     
      
      vec[i][0] = x;
      vec[i][1] = y;
      vec[i][2] = 5;

 
};

for(int i = 0; i < 50; i++)
    {
        cout << '{';
        for(int j = 0; j < 3 ;j++)
        {
            cout << vec[i][j] << " ";
            if(j!=2){
                cout << ',';
            }
        }
        cout << '}' << ',';
        cout << endl;
    };
  
 
 
return 0;
}
