#ifndef FILTER_H
#define FILTER_H

#include<string.h>
#include<iostream>

namespace Aris {
namespace Filter {


template<int LEN>
class CFilterFIR
{
public:
    CFilterFIR();
    ~CFilterFIR();
    void Filter(double inData,double& outData);
    void ResetReg();
    void FeedData(double inData);
    void GetData(double &outData);
    void SetCoef(double coef[LEN]);

private:
    double m_InData;
    double m_OutData;
    int m_CurrentIndex=0;
    int m_FilterLength=LEN;
    double m_Coef[LEN];
    double m_Reg[LEN];

};


template<int LEN>
CFilterFIR<LEN>::CFilterFIR()
{
    memset(m_Reg,0,m_FilterLength*sizeof(double));
    //memset(m_Coef,0,m_FilterLength*sizeof(double));

    m_CurrentIndex=0;
    m_InData=0;
    m_OutData=0;

}

template<int LEN>
CFilterFIR<LEN>::~CFilterFIR()
{
    return;
}

template<int LEN>
void CFilterFIR<LEN>::ResetReg()
{
    memset(this->m_Reg,0,this->m_FilterLength*sizeof(double));
    this->m_CurrentIndex=0;
    this->m_InData=0;
    this->m_OutData=0;

}

template<int LEN>
void CFilterFIR<LEN>::Filter(double inData, double &outData)
{
    m_InData=inData;
    //shift data
//    memcpy(&this->m_reg[1],&this->m_reg[0],(this->m_FilterLength-1)*sizeof(double));
    m_Reg[m_CurrentIndex]=inData;
    m_CurrentIndex++;
    if(m_CurrentIndex==LEN)
        this->m_CurrentIndex=0;
    outData=0;
    for(int i=0;i<m_FilterLength;i++)
    {
        outData+=m_Reg[
                (m_CurrentIndex-i)<0?
                    m_FilterLength+m_CurrentIndex-i:
                    m_CurrentIndex-i]*m_Coef[i];
    }
    m_OutData=outData;

}

template<int LEN>
void CFilterFIR<LEN>::GetData(double &outData)
{
    outData=m_OutData;
}

template<int LEN>
void CFilterFIR<LEN>::FeedData(double inData)
{
    m_InData=inData;

    //shift data
//    memcpy(&this->m_reg[1],&this->m_reg[0],(this->m_FilterLength-1)*sizeof(double));
    m_Reg[m_CurrentIndex]=inData;
    m_CurrentIndex++;
    if(m_CurrentIndex==LEN)
        m_CurrentIndex=0;
    m_OutData=0;
    for(int i=0;i<m_FilterLength;i++)
    {
        m_OutData+=m_Reg[
                (m_CurrentIndex-i)<0?
                    m_FilterLength+m_CurrentIndex-i:
                    m_CurrentIndex-i]*m_Coef[i];
    }

}

template<int LEN>
void CFilterFIR<LEN>::SetCoef(double coef[LEN])
{
    for(int i=0;i<LEN;i++)
    {
        m_Coef[i]=coef[i];
    }

}


// specialize a Filter for Rofo

class CFilterFIR_I: public CFilterFIR<41>
{
public:
    CFilterFIR_I()
    {

        double coef[41]={
                        0.01991188823604,  0.02053491965099,  0.02113804146569,  0.02171951567691,
                        0.02227765575763,  0.02281083311437,  0.02331748334669,  0.02379611228109,
                        0.02424530175286,  0.02466371510982,  0.02505010241382,  0.02540330531656,
                        0.02572226158817,   0.0260060092781,  0.02625369048986,  0.02646455475257,
                        0.02663796197434,  0.02677338496401,  0.02687041151014,  0.02692874600769,
                        0.02694821062531,  0.02692874600769,  0.02687041151014,  0.02677338496401,
                        0.02663796197434,  0.02646455475257,  0.02625369048986,   0.0260060092781,
                        0.02572226158817,  0.02540330531656,  0.02505010241382,  0.02466371510982,
                        0.02424530175286,  0.02379611228109,  0.02331748334669,  0.02281083311437,
                        0.02227765575763,  0.02171951567691,  0.02113804146569,  0.02053491965099,
                        0.01991188823604
                    };
        SetCoef(coef);
    }

};


}

}



#endif
