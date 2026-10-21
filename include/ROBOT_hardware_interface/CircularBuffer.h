#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

typedef struct CircularElement CircularElement;
typedef struct CircularBuffer CircularBuffer;

namespace FastFilters
{

    typedef struct CircularElement
    {
        CircularElement * NextElement;
        double Value;
    }CircularElement;

    typedef struct CircularBuffer
    {
        CircularElement* LastElement;
        double Sum;
        int Count;
        double Avg;
    };
    
 
    class CircularBufferClass
    {
        public:
            CircularBufferClass(int cnt);
            void AddElement(double value);
            double GetAvg();
        private:
            CircularBuffer buffer;
            
    };
    class AVGFilter: public CircularBufferClass
    {
        public:
            AVGFilter(int cnt);
    };
    
}


#endif /* CIRCULARBUFFER_H_ */
