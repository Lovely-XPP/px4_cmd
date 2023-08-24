#ifndef PRINTF_UTILITY_H
#define PRINTF_UTILITY_H
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <math.h>
#include <ftw.h>
#include <algorithm>

using namespace std;

// 颜色
#define RED "\033[0;1;31m"
#define GREEN "\033[0;1;32m"
#define YELLOW "\033[0;1;33m"
#define BLUE "\033[0;1;34m"
#define PURPLE "\033[0;1;35m"
#define DEEPGREEN "\033[0;1;36m"
#define WHITE "\033[0;1;37m"

// 指针
#define NO_POINTER "\033[?25l"
#define POINTER "\033[?25h"

/*  打印标题头  */
void print_head(std::string title_name)
{
    std::string title_sign = "*************************************************************";
    std::string title_space = "";
    std::string title;
    int title_sign_size = title_sign.size();
    int title_name_size = title_name.size();
    float space_num = (title_sign_size - title_name_size) / 2.0;
    for (int i = 0; i < floor(space_num - 1); i++)
    {
        title_space.append(" ");
    }
    std::string tmp_space = (space_num - floor(space_num) > 0.1) ? " " : "";
    title = title_sign + "\n" + "*" + title_space + title_name + title_space + tmp_space +
            "*" + "\n" + title_sign + "\n";
    cout << GREEN << title << WHITE << endl;
}

/*  打印标题  */
void print_title(std::string title_name, std::vector<string> title_content)
{
    print_head(title_name);
    std::string choice;
    int choice_num = 0;
    for (auto item = title_content.begin(); item != title_content.end(); ++item)
    {
        choice = choice + to_string(choice_num) + ". " + *item + "\n";
        choice_num++;
    }
    cout << choice << endl;
}

void Error(string msg)
{
    cout << RED << "[ERROR] " + msg << WHITE << endl;
}

void Warning(string msg)
{
    cout << YELLOW << "[ WARN] " + msg << WHITE << endl;
}

void Info(string msg)
{
    cout << GREEN << "[ INFO] " + msg << WHITE << endl;
}

void get_cmd_output(const char *cmd, std::string &result)
{
    FILE *pipe = popen(cmd, "r");
    if (!pipe)
    {

        printf("popen error\n");
        return;
    }
    size_t ret = 0;
    char *buf = nullptr;
    size_t len = 1024;
    while ((ret = getline(&buf, &len, pipe)) != -1)
    {
        result.append(buf);
    }
    return;
};

void strip(std::string &s, const char *str = " ")
{
    int str_len = strlen(str);
    std::string first = "";
    std::string last = "";
    if (s.size() == 0)
    {
        return;
    }
    // 去掉字符串开头的空格
    first = s.substr(0,str_len);
    while (!first.compare(str))
    {
        s.erase(0, s.find_first_not_of(str));
        first = s.substr(0, str_len);
    }
    if (s.size() == 0)
    {
        return;
    }
    // 去掉字符串末尾的空格
    last = s.substr(s.length() - str_len, s.length());
    while (!last.compare(str))
    {
        s.erase(s.find_last_not_of(str) + str_len);
        last = s.substr(s.length() - str_len, s.length());
    }
}

vector<string> files = {};

int add_files(const char *fpath, const struct stat *sb, int typeflag)
{
    string file = fpath;
    files.push_back(file);
    return 0;
}

vector<string> get_files(string dir)
{
    ftw(dir.c_str(), &add_files, 1);
    files.erase(files.begin());
    for (auto item = files.begin(); item != files.end(); item++)
    {
        std::string file = *item;
        *item = file.erase(0, dir.length());
    }
    sort(files.begin(), files.end());
    return files;
};
#endif