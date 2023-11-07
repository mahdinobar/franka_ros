#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
using namespace std;
int main()
{


  ifstream inputfile("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentPosition_log.txt");
  if (!inputfile.is_open())
  {
    cout<<"error"<<endl;
  }


  static const int MAX_ROWS = 5175;
  static const int MAX_COLUMNS = 3;
  double matrix[MAX_ROWS][MAX_COLUMNS];
  //...
  for (int row = 0; row < MAX_ROWS; ++ row)
  {
      std::string row_text;
      std::getline(inputfile, row_text);
      std::istringstream row_stream(row_text);
      for (int column = 0; column < MAX_COLUMNS; ++column)
      {
        double number;
        char delimiter;
        row_stream >> number >> delimiter;
        matrix[row][column] = number;
      }
  }
  for (int r = 0; r < MAX_ROWS; r++) {
    for (int c = 0; c < MAX_COLUMNS; c++) {
      cout << matrix[r][c] << "\t";
    }
    cout << endl;
  }

    inputfile.close();
    return 0;
  }



//#include <fstream>
//#include <iostream>
//
//using namespace std;
//
//int main() {
//  int row = 5175;
//  int col = 3;
//
//  double myArray[row][col];
//
//  // Opening the file
//  ifstream inputfile("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentPosition_log.txt", ios::in);
//
//  if (!inputfile.is_open()) cout << "Error opening file";
//
//  // Defining the loop for getting input from the file
//
//  for (int r = 0; r < row; r++)  // Outer loop for rows
//  {
//    for (int c = 0; c < col; c++)  // inner loop for columns
//    {
//      inputfile >> myArray[r][c];  // Take input from file and put into myArray
//    }
//  }
//
//  for (int r = 0; r < row; r++) {
//    for (int c = 0; c < col; c++) {
//      cout << myArray[r][c] << "\t";
//    }
//    cout << endl;
//  }
//}
