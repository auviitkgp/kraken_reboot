#ifndef CONTROLPARAMETERS_H
#define CONTROLPARAMETERS_H

#include "iostream"
#include "fstream"
#include "ros/ros.h"

namespace kraken_controller{
    class ControlParameters{
    public:
        ControlParameters(int row = 6, int cols = 30);
        ~ControlParameters();
        void load(const std::string &filename);
        void write(std::fstream *file);
        void write(std::ostream &out);
        double **getGain();
        double *getOffset();
        const std::string getName();
        int getRows();
        int getColumns();
        
    protected:
        double **_gain;
        double * _offset;
        int _row;
        int _cols;
        std::string _name;
    };
}

#endif
