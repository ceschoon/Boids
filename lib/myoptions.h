#ifndef MYOPTIONS_H
#define MYOPTIONS_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

using namespace std;


bool detectOption(int argc, char **argv, string name);

void readOption(int argc, char **argv, string name, int &value);
void readOption(int argc, char **argv, string name, double &value);
void readOption(int argc, char **argv, string name, string &value);


bool detectOption(int argc, char **argv, string name)
{
	string opttomatch = "--"+name;
	
	for (int i=0; i<argc; i++)
	{
		string opt = argv[i];
		string optid = opt.substr(0,name.size()+2);
		
		if (optid==opttomatch) return true;
	}
	
	return false;
}


void readOption(int argc, char **argv, string name, int &value)
{
	string opttomatch = "--"+name+"=";
	
	for (int i=0; i<argc; i++)
	{
		string opt = argv[i];
		string optid = opt.substr(0,name.size()+3);
		
		if (optid==opttomatch)
		{
			try 
			{
				string optval = opt.substr(name.size()+3, opt.size()-name.size()-3);
				value = stoi(optval);
			}
			catch (...) {cout << "Error reading option: " << opt << endl;}
		}
	}
}


void readOption(int argc, char **argv, string name, double &value)
{
	string opttomatch = "--"+name+"=";
	
	for (int i=0; i<argc; i++)
	{
		string opt = argv[i];
		string optid = opt.substr(0,name.size()+3);
		
		if (optid==opttomatch)
		{
			try
			{
				string optval = opt.substr(name.size()+3, opt.size()-name.size()-3);
				value = stod(optval);
			}
			catch (...) {cout << "Error reading option: " << opt << endl;}
		}
	}
}


void readOption(int argc, char **argv, string name, string &value)
{
	string opttomatch = "--"+name+"=";
	
	for (int i=0; i<argc; i++)
	{
		string opt = argv[i];
		string optid = opt.substr(0,name.size()+3);
		
		if (optid==opttomatch)
		{
			try
			{
				string optval = opt.substr(name.size()+3, opt.size()-name.size()-3);
				value = optval;
			}
			catch (...) {cout << "Error reading option: " << opt << endl;}
		}
	}
}


#endif