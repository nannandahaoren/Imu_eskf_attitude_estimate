#include <iostream>
#include <fstream>

using namespace std;

std::string CSV_PATH = "./data.csv";
ofstream csv;

int frame_num = 0;

struct Name
{
    int age;
    float height;
    float weight;
    int score;
};

int main()
{
    Name Zhangsan{30, 1.75, 78, 98};
    csv.open(CSV_PATH);

    while (true)
    {
        if (csv.is_open())
        {
            frame_num++;
            if (frame_num == 1)
            {
                csv << "age"
                    << ","
                    << "height(m)"
                    << ","
                    << "weight(kg)"
                    << ","
                    << "score"
                    << "\n";
            }
            else
            {
                csv << Zhangsan.age << "," << Zhangsan.height << "," << Zhangsan.weight << "," << Zhangsan.score
                    << "\n";
            }
        }
        if (frame_num > 10000) {
            csv.close();
        }
    }

    return 0;
}
