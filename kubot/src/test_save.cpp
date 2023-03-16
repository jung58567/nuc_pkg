#include <iostream>
#include <fstream>
#include <string>

double thread_t[1000] = {};
int cnt = 0;
int max_cnt = 50;

using namespace std;

void save(){
    string line;
    ofstream file;
    file.open("time.txt",ios_base::app);

    if(file.is_open()) {
		for(int i=1;i<=cnt;i++)
        {
        cout<<"save ="<<i<<"="<<thread_t[i-1]<<endl;
        file <<thread_t[i-1]<<"\n";
        }
		file.close();
	}
    else {
		cout << "error" << endl;	
	}
}
int main()
{
    cnt=0;
    while (1)
    {
        cout<<"cnt =" <<cnt<<endl;
        cout<<"max =" <<max_cnt<<"\n"<<endl;
        thread_t[cnt-1]=cnt;
        if(cnt == max_cnt)
        {
            save();
            cnt++;
            break;
        }
        else
        {
            cnt++;
        }
    }
    return 0;
}