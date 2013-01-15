#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>

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
 ifstream tech_file;
 double nlut, nff, nmux, num_input, num_out, num_cross, num_l1, num_l4, num_long, num_swi, nsmb, num_direct; 
 map<int, int> x_table;
 map<int, int> y_table;
 int track_num, length, current_type;
 vector<string> v;
 num_l1=0;
 num_l4=0;
 num_long =0;
 num_swi=0;
 num_direct =0;
 
 bool is_leak = false;
 if (argc != 6&&argc!=4&& argc!=7)
   {cout << "error in number of parameters" << endl;
   return 1;}

 input_file.open(argv[1]);
 if(!input_file.is_open())
   {cout<< "Cannot open parameter file: " << argv[1] << "\n";
   return 1;
   }
 
 while (!input_file.eof())
   {getline(input_file, oneline);
     if (oneline.find_first_not_of(" ")==string::npos)
       continue;
     v = get_word(oneline);
     // cout << oneline << endl;
     if(v[0] == "num_LUT")
       nlut = stod(v[1]);
     if(v[0] == "num_FF")
       nff = stod(v[1]);
     if(v[0] == "num_MUX")
       nmux = stod(v[1]);
     if(v[0] == "num_input")
       num_input = stod(v[1]);
     if(v[0] == "num_output")
       num_out = stod(v[1]);
     if(v[0] == "length_1")
       num_l1 = stod(v[1]);
     if(v[0] == "length_4")
       num_l4 = stod(v[1]);
     if(v[0] == "length_long")
       num_long = stod(v[1]);
     if(v[0] == "num_switch")
       num_swi = stod(v[1]);
     if(v[0] == "num_SMB")
       nsmb = stod(v[1]);
     if(v[0] == "num_cross")
       num_cross = stod(v[1]);
   }
 input_file.close();
  
  tech_file.open(argv[3]);
 if(!tech_file.is_open())
   {cout<< "Cannot open parameter file: " << argv[3] << "\n";
   return 1;
   }
 
 double LUT,MUX,CROS,IOBAR,OUTBAR,Dff,Switch,direct,Length1,Length4,Long,mem;
 double Switch_leak,Length1_leak,Length4_leak,direct_leak,Long_leak,mem_leak,sram_leak,leak_per_smb,clock;
 while (!tech_file.eof())
   {getline(tech_file, oneline);
     if ((oneline.find_first_not_of(" ")==string::npos)||(oneline.find("\\")!=string::npos))
       continue;
     v = get_word(oneline);
     // cout << oneline << endl;
     if(v[0] == "LUT")
       {LUT = stod(v[1]);
	 cout << LUT << endl;}
     if(v[0] == "MUX")
       {MUX = stod(v[1]);
	 cout << MUX << endl;}
     if(v[0] == "CROS")
       {CROS = stod(v[1]);
	 cout << CROS << endl;}
     if(v[0] == "IOBAR")
       {IOBAR = stod(v[1]);
	 cout << IOBAR << endl;}
     if(v[0] == "OUTBAR")
       {OUTBAR = stod(v[1]);
	 cout << OUTBAR << endl;}
     if(v[0] == "Dff")
       {Dff = stod(v[1]);
	 cout << Dff << endl;}
     if(v[0] == "Switch")
       {Switch = stod(v[1]);
	 cout << Switch << endl;}
     if(v[0] == "direct")
       {direct = stod(v[1]);
	 cout << direct << endl;}
     if(v[0] == "Length1")
       {Length1 = stod(v[1]);
	 cout << Length1 << endl;}
     if(v[0] == "Length4")
       {Length4 = stod(v[1]);
	 cout << Length4 << endl;}
     if(v[0] == "Long")
       {Long = stod(v[1]);
	 cout << Long << endl;}
     if(v[0] == "mem")
       {mem = stod(v[1]);
	 cout << mem << endl;}
     if(v[0] == "Switch_leak")
       {Switch_leak = stod(v[1]);
	 cout << Switch_leak << endl;}
     if(v[0] == "Length1_leak")
       {Length1_leak = stod(v[1]);
	 cout << Length1_leak << endl;}
     if(v[0] == "Length4_leak")
       {Length4_leak = stod(v[1]);
	 cout << Length4_leak << endl;}
     if(v[0] == "direct_leak")
       {direct_leak = stod(v[1]);
	 cout << direct_leak << endl;}
     if(v[0] == "Long_leak")
       {Long_leak = stod(v[1]);
	 cout << Long_leak << endl;}
     if(v[0] == "mem_leak")
       {mem_leak = stod(v[1]);
	 cout << mem_leak << endl;}
     if(v[0] == "sram_leak")
       {sram_leak = stod(v[1]);
	 cout << sram_leak << endl;}
     if(v[0] == "leak_per_smb")
       {leak_per_smb = stod(v[1]);
	 cout << leak_per_smb << endl;}
     if(v[0] == "clock")
       {clock = stod(v[1]);
	 cout << clock << endl;}
   }
 tech_file.close();

  if (argc==6)
    {cout << argc << endl;
    tx_file.open(argv[4]);}
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
 
  srand((unsigned) time(NULL));
  static const double r_max=double(RAND_MAX)+double(1);
  double switch_num = rand()/r_max + 0.5;
  //cout << switch_num << endl;
  bool switch_on = false;
  
  if((argc==7)&&(argv[6]=="-r"))
    switch_on = true;
  if(argc==6)
    ty_file.open(argv[5]);
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
 
  rt_file.open(argv[2]);
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

 cout << "Please input the delay:" << endl;
 string word;
 getline(cin, word);
 double freq = 1/stod(word);
 cout << "The frequency is " << freq << '\n';

 cout << "Please input num of stage:" << endl;
 getline(cin, word);
 int num_stage = stoi(word);

 cout << "Please input the FPGA size:" << endl;
 getline(cin, word);
 int size = stoi(word);

 double dyna_pow_logic, leak_pow_logic, reconf_pow, reconf_leak_pow, dyn_inter, leak_inter, total_leak;
 
 double another_leak, another_leak1, total_mem_leak;

 cout << "length-1: " << num_l1 << '\n';
 cout << "length-4: " << num_l4 << '\n';
 cout << "long: " << num_long << '\n';
 cout << "swi: " << num_swi << '\n';
 cout << "direct: " << num_direct << '\n';

 dyna_pow_logic = nlut*LUT+nff*Dff;

 dyn_inter = num_input*IOBAR+num_out*OUTBAR+num_l1*Length1+num_l4*Length4+num_long*Long+num_swi*Switch+nmux*MUX+num_cross*CROS + num_direct*direct; 
 //leak_pow_logic = nlut*LUT_leak+nff*Dff_leak+nmux*MUX_leak+num_cross*CROS_leak;

 //leak_inter =num_input*IOBAR_leak+num_out*OUTBAR_leak+num_l1*Length1_leak+num_l4*Length4_leak+num_long*Long_leak+num_swi*Switch_leak;
 reconf_pow = nsmb*mem;
 //reconf_leak_pow = nsmb*mem_leak;

 total_leak = nsmb*leak_per_smb+ num_l1*Length1_leak+num_l4*Length4_leak+num_long*Long_leak+num_swi*Switch_leak + num_direct*direct_leak;
 //cout << "total leak" << endl;
 //cout << nsmb*leak_per_smb << " " << num_l4*Length4_leak << " " << num_swi*Switch_leak << '\n';
 another_leak = total_leak-nsmb*mem_leak;

 dyna_pow_logic = dyna_pow_logic/num_stage*freq/0.5;
 cout << "The dynamic pow logic: " <<  dyna_pow_logic << '\n';
 
 reconf_pow = reconf_pow/num_stage*freq/0.5;
 cout << "The dyn reconf pow: " << reconf_pow  << '\n';
 
 dyn_inter = dyn_inter/num_stage*freq/0.5; 
 cout << "The dynamic pow inter: " << dyn_inter << '\n';
 total_leak = total_leak/num_stage;
 total_mem_leak = nsmb*mem_leak/num_stage;
 another_leak = another_leak/num_stage;

 cout << "total leak pow: " << total_leak << '\n';
 cout << "memory leak" << total_mem_leak<< endl;
 cout << "leak pow without memory for no_folding: " << another_leak << '\n';
 double clock_pow = size*clock*freq/0.5; 
 cout << "clock pow: " << clock_pow <<'\n';
 double total_pow, total_pow1, total_pow2;
 if (!switch_on)
   total_pow = clock_pow + total_leak + dyn_inter + reconf_pow + dyna_pow_logic;
 else
   total_pow = clock_pow + total_leak + reconf_pow + switch_num*(dyn_inter+dyna_pow_logic);

 cout << "total pow: " << total_pow << '\n';
 if (!switch_on)
   total_pow1 = clock_pow + another_leak+dyn_inter+dyna_pow_logic;
 else
   total_pow1 = clock_pow + another_leak+switch_num*(dyn_inter+dyna_pow_logic);

 cout << "total pow for no_folding: " << total_pow1 << '\n';

if (!switch_on)
   total_pow2 = clock_pow + total_leak+dyn_inter+dyna_pow_logic;
 else
   total_pow2 = clock_pow + total_leak+switch_num*(dyn_inter+dyna_pow_logic);

 cout << "total pow consider memory leak: " << total_pow2 << '\n';
 return 0;
}
