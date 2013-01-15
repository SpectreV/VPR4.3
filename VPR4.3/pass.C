#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace std;

double stod(string word)
{stringstream s;
 double result;
 s << word;
 s >> result;
 s.clear();
 return result;
}

int stoi(string word)
{stringstream s;
 int result;
 s << word;
 s >> result;
 s.clear();
 return result;
}

string itos (int inter)
{stringstream s;
 string ss;
 s << inter;
 s >> ss;
 s.clear();
 return ss;
}

vector<string> get_word(string line)
{int str_begin, str_end, str_temp, word_begin;
 string sub_str, word;
 vector<string> v;
        str_begin = line.find_first_not_of(" ");
        str_end = line.find_last_not_of(" ");
        sub_str = line.substr(str_begin, str_end+1);
	while (sub_str.find_first_not_of(" ") != string::npos)
	    {str_temp = sub_str.find_first_of (" ");
	    word_begin = sub_str.substr(str_temp+1, str_end).find_first_not_of (" ");
             word = sub_str.substr(0, str_temp);
             v.push_back(word);
             str_begin = str_temp+1+word_begin;
             sub_str = sub_str.substr(str_begin, str_end);
	     if (sub_str.find_first_of(" ") == string :: npos)
	       {
		 if(sub_str.find_first_not_of(" ") != string::npos)
		 {v.push_back(sub_str);
		 break;}		 
	       } 
	    }
        return v;
}

#define LENGTH_1 0
#define LENGTH_4 1
#define LONGWIRE 2
#define CHANX 3
#define CHANY 4
#define NOCHAN 5

int main (int argc, char *argv[])
{string oneline;
 string temp;
 ifstream input_file;
 ifstream rt_file;
 ifstream tx_file;
 ifstream ty_file;
 double nlut, nff, nmux, num_input, num_out, num_cross, num_l1, num_l4, num_long, num_swi, nsmb, num_direct; 
 map<int, int> x_table;
 map<int, int> y_table;
 int track_num, length, current_type, ;
 vector<string> v;
 num_l1=0;
 num_l4=0;
 num_long =0;
 num_swi=0;
 num_direct =0;
 
 bool is_leak = false;
 if (argc!=4)
   {cout << "error in number of parameters" << endl;
   return 1;}
 
  if (argc==4)
    {
    tx_file.open(argv[2]);}
  else
    {string xfile = "x.echo";
     tx_file.open(xfile.c_str());
     //  cout << "opne file " << endl;
    }
 if(!tx_file.is_open())
   {cout<< "Cannot open file: " << argv[3] << "\n";
   return 1;
   }
  while (!tx_file.eof())
    {getline(tx_file, oneline);
    if (oneline.find_first_not_of(" ")==string::npos)
      continue;
    v = get_word(oneline);
    //cout << oneline << endl;
    if(v[0] == "Track:")
      {temp = v[1];
      temp = temp.substr(0,temp.length()-1);
      track_num = stoi(temp);
      //cout << "track number " << temp << '\n';
      }
    if(v[0] == "Length:")
      {temp = v[1];
       temp = temp.substr(0,temp.length()-1);
       length = stoi(temp);
       // cout << "length " << temp << '\n'; 
       if (length ==1)
	 x_table[track_num] = LENGTH_1;
       if (length > 1 && length <=4)
	 x_table[track_num] = LENGTH_4;
       if (length >4)
	 x_table[track_num] = LONGWIRE;
      }
    }
  tx_file.close();

  if(argc==4)
    ty_file.open(argv[3]);
  else
    ty_file.open("y.echo");
 if(!ty_file.is_open())
   {cout<< "Cannot open file: " << argv[4] << "\n";
   return 1;
   }
  while (!ty_file.eof())
    {getline(ty_file, oneline);
    if (oneline.find_first_not_of(" ")==string::npos)
      continue;
    v = get_word(oneline);
    // cout << oneline << endl;
    if(v[0] == "Track:")
      {temp = v[1];
      temp = temp.substr(0,temp.length()-1);
      track_num = stoi(temp);
      //cout << "track number " << temp << '\n';
      }
    if(v[0] == "Length:")
      {temp = v[1];
       temp = temp.substr(0,temp.length()-1);
       length = stoi(temp);
       //cout << "length " << temp << '\n';
       if (length ==1)
	 y_table[track_num] = LENGTH_1;
       if (length >1 && length <=4)
	 y_table[track_num] = LENGTH_4;
       if (length >4)
	 y_table[track_num] = LONGWIRE;
      }
    }
  ty_file.close();
 
  rt_file.open(argv[1]);
 if(!rt_file.is_open())
   {cout<< "Cannot open route file: " << argv[2] << "\n";
   return 1;
   }
 while (!rt_file.eof())
   {getline(rt_file, oneline);
   if (oneline.find_first_not_of(" ")==string::npos)
      continue;
   v = get_word(oneline);
   // cout << oneline << endl;
   if(v[0] == "CHANX")
     {if ((current_type = CHANY)||(current_type ==CHANX))
       {num_swi++;
       //cout << "chan x " << num_swi << '\n';
       current_type = CHANX;}
     if(v.size()==6)
       {track_num = stoi(v[5]);
       //cout << "track_num " << track_num <<'\n';
       }
     else if (v.size()==4)
       {track_num = stoi(v[3]);
       //cout << "track_num " << track_num <<'\n';
       }
     else
       {cout << "format wrong!" << endl;
       return 1;}
     //track_num = stoi(v[5]);
     int type = x_table[track_num];
     //cout << "track " << track_num << '\n';
     if (type == LENGTH_1)
       {num_l1++;
       //cout << "length -1 " << num_l1 << '\n';
       }
     if (type == LENGTH_4)
	 {num_l4++;
	 //cout << "length -4 " << num_l4 << '\n';
	 }
     if (type == LONGWIRE)
       {num_long++;
       //cout << "long " << num_long << '\n';
       }
     }
   if ((v[0] == "DIREX" )||(v[0]=="DIREY"))
     num_direct ++;
   if(v[0] == "CHANY")
     {if ((current_type = CHANX)||(current_type==CHANY))
       {num_swi++;
       //cout << "num of switch " << num_swi << '\n';
       current_type = CHANY;}
     if(v.size()==6)
       {track_num = stoi(v[5]);
       //cout << "track_num " << track_num <<'\n';
       }
     else if (v.size()==4)
       {track_num = stoi(v[3]);
       //cout << "track_num " << track_num <<'\n';
       }
     else
       {cout << "format wrong!" << endl;
       return 1;}
     int type = y_table[track_num];
     if (type == LENGTH_1)
       {num_l1++;
       //cout << "length-1 " << num_l1 << '\n';
       }
     if (type == LENGTH_4)
       {num_l4++;
       //cout << "length-4 " << num_l4 << '\n';
       }
     if (type == LONGWIRE)
       {num_long++;
       //cout << "long wire " << num_long << '\n';
       }
     }
   if((v[0]!="CHANX")&&(v[0]!="CHANY"))
     current_type = NOCHAN;
   }
 rt_file.close();

 cout << "length-1 wire: " << num_l1 << '\n';
 cout << "length-4 wire: " << num_l4 << '\n';
 cout << "long wire: " << num_long << '\n';
 cout << "direct link: " << num_direct << '\n';

 return 0;
}
