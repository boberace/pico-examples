#ifndef CINDEX_H
#define CINDEX_H

class cindex{

private:
    int _top_num;// circular index top number
    int _idx = 0;// current index value

public:
    cindex(int);
    operator int();
    void operator = (int x);
    int operator ++ (void);// prefix
    int operator ++ (int); // postfix
    int operator -- (void);// prefix
    int operator -- (int); // postfix
 
};
#endif


