#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
using namespace std;
int main()
{


  ifstream inputfile_r_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentPosition_log.txt");
  if (!inputfile_r_star.is_open())
  {
    cout<<"Error reading desired position"<<endl;
  }

  ifstream inputfile_v_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentVel_log.txt");
  if (!inputfile_v_star.is_open())
  {
    cout<<"Error reading desired velocity"<<endl;
  }

  static const int MAX_ROWS = 5175;
  static const int MAX_COLUMNS = 3;
  double r_star[MAX_ROWS][MAX_COLUMNS];
  double v_star[MAX_ROWS][MAX_COLUMNS];

  for (int row = 0; row < MAX_ROWS; ++ row)
  {
      std::string row_text_r;
      std::getline(inputfile_r_star, row_text_r);
      std::istringstream row_stream_r(row_text_r);
      std::string row_text_v;
      std::getline(inputfile_v_star, row_text_v);
      std::istringstream row_stream_v(row_text_v);
      for (int column = 0; column < MAX_COLUMNS; ++column)
      {
        double number_r;
        double number_v;
        char delimiter;
        row_stream_r >> number_r >> delimiter;
        r_star[row][column] = number_r;
        row_stream_v >> number_v >> delimiter;
        v_star[row][column] = number_v;
      }
  }
  for (int r = 0; r < MAX_ROWS; r++) {
    for (int c = 0; c < MAX_COLUMNS; c++) {
      cout << r_star[r][c] << "\t";
    }
    cout << endl;
  }

    inputfile_r_star.close();
    return 0;
  }
