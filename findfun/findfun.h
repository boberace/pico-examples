#ifndef FINDFUN_H
#define FINDFUN_H

#include <algorithm>
#include <vector>
#include <math.h>

using std::sort;
using std::find;
using std::vector;

class findfun{
public:
void begin( float CON_PITCH = 440.0, // concert pitch
            float SAMP_FREQ = 2.5*1000000, // sample edge frequency (hz)      
            int LOW_MIDI_INDEX = 38,
            int HIGH_MIDI_INDEX = 66+1,
            int NUM_CORR_SHIFTS = 20
            );

int find_midi_cent(const vector<int> &sample_edges); 


private:

    int low_midi_index;
    int high_midi_index;
    int num_corr_shifts;
    int samp_freq_factor;
    vector<vector<int>> midi_cent_test_edges;
    int find_closest_period(const vector<int> &sample_edges, 
                            const vector<int> test_edges_idxs);
    int get_index_vector_int(vector<int> v, int K);

};
#endif



