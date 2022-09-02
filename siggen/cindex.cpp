#include "cindex.h"
// #include <stdexcept>
using namespace std;
cindex::cindex(int x ) : _top_num{ x } {

    if (x < 2){
        // throw invalid_argument(" cindex top is less than 2");
        _top_num = 2;
    }

}

cindex::operator int() {return _idx;}

void cindex::operator = (int x ){ _idx = x % _top_num; }

int cindex::operator ++ (void){   

    _idx ++;
    _idx = _idx % _top_num;
    return _idx;
}


int cindex::operator ++ (int){    

    int x = _idx; 
    ++ _idx;
    _idx = _idx % _top_num;
    return x;
}


int cindex::operator -- (void){   

    if (_idx == 0){
        _idx = _top_num - 1;
    } else {
        _idx--;
    }
    return _idx;
}

int cindex::operator -- (int){    

    int x = _idx; 

    if(_idx == 0){
        _idx = _top_num - 1;
    } else {

        -- _idx;
    }

    return x;
}

