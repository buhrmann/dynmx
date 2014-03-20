// ************************************************************
// A class for continuous-time recurrent neural networks
//
// RDB 
//  8/94 Created
//  12/98 Optimized integration
//  1/08 Added table-based fast sigmoid w/ linear interpolation
//
// To Do
//   1. Could optimize a bit further by making StepSize
//      a CTRNN parameter and prescaling RTaus and
//      by providing versions of EulerStep and RK4Step
//      that ignore gains, but it's not clear that the
//      few percent speed increase is really worth it.
// ************************************************************

#include "CTRNN.h"
#include "Random.h"
#include <iomanip>

namespace dmx 
{

// A fast sigmoid implementation using a table w/ linear interpolation
//----------------------------------------------------------------------------------------------------------------------
#ifdef FAST_SIGMOID
int SigTableInitFlag = 0;
double SigTab[SigTabSize];

//----------------------------------------------------------------------------------------------------------------------
void initSigmoidTable(void)
{
  if (!SigTableInitFlag) 
  {
    double DeltaX = SigTabRange / (SigTabSize - 1);
    for (int i = 0; i <= SigTabSize - 1; i++)
      SigTab[i] = sigma(i * DeltaX);
    SigTableInitFlag = 1;
  }
}

//----------------------------------------------------------------------------------------------------------------------
double fastsigmoid(double x)
{
  if (x >= SigTabRange) 
    return 1.0;
  if (x < 0) 
    return 1.0 - fastsigmoid(-x);
    
  double id;
  double frac = modf(x * (SigTabSize - 1) / SigTabRange, &id);
  int i = (int)id;
  double y1 = SigTab[i], y2 = SigTab[i + 1];
 
  return y1 + (y2 - y1) * frac;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
// The constructor
//----------------------------------------------------------------------------------------------------------------------
CTRNN::CTRNN(int newsize)
{
#ifdef FAST_SIGMOID
  initSigmoidTable();
#endif
  
	size = newsize;
  
  // allocate
	states = new double[size];
  outputs = new double[size];
  biases = new double[size];
  gains = new double[size];
  taus = new double[size];
  Rtaus = new double[size];
  inputs = new double[size];
  externalinputs = new double[size];
  m_updateFunctions = new UpdateFunction[size];
  m_activationFunctions = new ActivationFunction[size];
  
  weights = new double*[size];
  for(int i = 0; i < size; i++)
  {
    weights[i] = new double[size];
    for(int j = 0; j < size; j++)
    {
      weights[i][j] = 0.0;
    }
  }
  
  // initialize
  std::fill(states, states + size, 0.0);
  std::fill(outputs, outputs + size, 0.0);
  std::fill(biases, biases + size, 0.0);
  std::fill(inputs, inputs + size, 0.0);
  std::fill(externalinputs, externalinputs + size, 0.0);
  std::fill(gains, gains + size, 1.0);
  std::fill(taus, taus + size, 1.0);
  std::fill(Rtaus, Rtaus + size, 1.0);
  
  std::fill(m_updateFunctions, m_updateFunctions + size, &CTRNN::updateNeuron);
  std::fill(m_activationFunctions, m_activationFunctions + size, &sigmoid);
  
  setGlobalActivationFunction(kAF_Sigmoid);
}


// The destructor
//----------------------------------------------------------------------------------------------------------------------
CTRNN::~CTRNN()
{
  //TODO
  delete [] states;
  delete [] outputs;
  delete [] biases;
  delete [] gains;
  delete [] taus;
  delete [] Rtaus;
  delete [] inputs;
  delete [] externalinputs;
  
  delete [] m_updateFunctions;
  delete [] m_activationFunctions;
  
  for(int i = 0; i < size; i++)
  {
    delete [] weights[i];
  }
  
  delete [] weights;
}
  
//----------------------------------------------------------------------------------------------------------------------    
CTRNN::UpdateFunction CTRNN::getUpdateFunction(UpdFuncName name)
{
  switch (name)
  {
    case kUF_Neuron:
    default:
      return &CTRNN::updateNeuron;
      break;
    case kUF_Input:
      return &CTRNN::updateInput;
  }
}  

//----------------------------------------------------------------------------------------------------------------------    
CTRNN::ActivationFunction CTRNN::getActivationFunction(ActFuncName name)
{
  switch (name)
  {
    case kAF_Sigmoid:
    default:
      return &sigmoid;      
    case kAF_Sine:
      return &sineActivation;
    case kAF_Linear:
      return &linearActivation;
    case kAF_Identity:
      return &identity;
  }
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNN::setInputNeuron(int i)
{
  setUpdateFunction(i, kUF_Input);
  setActivationFunction(i, kAF_Identity);
  setBias(i, 0.0);
}
    

// Randomize the states or outputs of a circuit.
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::randomizeState(double lb, double ub)
{
	for (int i = 0; i < size; i++)
    setState(i, UniformRandom(lb, ub));
}

void CTRNN::randomizeState(double lb, double ub, RandomState &rs)
{
	for (int i = 0; i < size; i++)
    setState(i, rs.UniformRandom(lb, ub));
}

void CTRNN::randomizeOutput(double lb, double ub)
{
	for (int i = 0; i < size; i++)
    setOutput(i, UniformRandom(lb, ub));
}

void CTRNN::randomizeOutput(double lb, double ub, RandomState &rs)
{
	for (int i = 0; i < size; i++)
    setOutput(i, rs.UniformRandom(lb, ub));
}

void CTRNN::randomizeBiases(double lb, double ub)
{
	for (int i = 0; i < size; i++)
    setBias(i, UniformRandom(lb, ub));
}

void CTRNN::randomizeTimeConstants(double lb, double ub)
{
	for (int i = 0; i < size; i++)
    setTimeConstant(i, UniformRandom(lb, ub));
}

void CTRNN::randomizeWeights(double lb, double ub)
{
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
      setWeight(i, j, UniformRandom(lb, ub));
}

void CTRNN::randomizeWeights(double lb, double ub, RandomState &rs)
{
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
        setWeight(i, j, rs.UniformRandom(lb, ub));
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNN::zeroStates()
{
	for (int i = 0; i < size; i++)
    setStateDynamic(i, 0.0);
}


// Integrate a circuit one step using Euler integration.
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::update(double stepsize)
{
  // Update the state of all neurons.
  for (int i = 0; i < size; i++) 
  {
    inputs[i] = gains[i] * externalinputs[i];
    for (int j = 0; j < size; j++) 
    {
      inputs[i] += weights[j][i] * outputs[j];
    }
    
    states[i] += stepsize * Rtaus[i] * (inputs[i] - states[i]);
  }
  
  // Update the outputs of all neurons.
  for (int i = 0; i < size; i++)
  {
    outputs[i] = (*m_activationFunction)(states[i] + biases[i]);
  }
}
  
// Integrate a circuit using dynamic choice of update and activation function
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::updateDynamic(double stepsize)
{
  // Synchronously update the state of all neurons.
  for (int i = 0; i < size; i++) 
  {
    // Call this neuron's pointer to member function
    (this->*(m_updateFunctions[i]))(i, stepsize);    
  }
  
  // Synchronously update the outputs of all neurons.
  for (int i = 0; i < size; i++)
  {
    outputs[i] = (*m_activationFunctions[i])(states[i] + biases[i]);
  }
}  

//----------------------------------------------------------------------------------------------------------------------  
void CTRNN::updateNeuron(int i, double stepsize)
{
  // Update the state of all neurons.
  inputs[i] = gains[i] * externalinputs[i];
  for (int j = 0; j < size; j++) 
  {
    inputs[i] += weights[j][i] * outputs[j];
  }
  
  states[i] += stepsize * Rtaus[i] * (inputs[i] - states[i]);
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNN::updateInput(int i, double stepsize)
{
  states[i] = gains[i] * externalinputs[i];
}

// Set the biases of the CTRNN to their center-crossing values
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::setCenterCrossing(void)
{
  double InputWeights, ThetaStar;
  
  for (int i = 0; i < getSize(); i++) 
  {
      // Sum the input weights to this neuron
      InputWeights = 0;
      for (int j = 0; j < size; j++)
          InputWeights += weights[j][i];
          
      // Compute the corresponding ThetaStar
      ThetaStar = -InputWeights / 2;
      setBias(i, ThetaStar);
  }
}

//----------------------------------------------------------------------------------------------------------------------  
double CTRNN::getWeightSum() const
{
  double sum = 0;
  for (int i = 0; i < size; i++) 
  {
    for (int j = 0; j < size; j++)
    {
      sum += std::fabs(weights[i][j]);
    }
  }
  return sum;
}
  
//----------------------------------------------------------------------------------------------------------------------  
double CTRNN::getLargestWeight() const
{
  double w = -666;
  for (int i = 0; i < size; i++) 
  {
    for (int j = 0; j < size; j++)
    {
      if(weights[i][j] > w)
        w = weights[i][j];
    }
  }
  return w;
}
  
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::knockOut(int n)
{
  for (int i = 0; i < size; ++i)
  {
    setWeight(i, n, 0);
    setWeight(n, i, 0);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNN::leakWeights(float tau, float cutoff)
{
  for (int i = 0; i < size; i++)
  {
    for (int j = 0; j < size; j++)
    {
      weights[i][j] *= tau;
      if(weights[i][j] < cutoff)
        weights[i][j] = 0.0f;
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNN::toXml(ci::XmlTree& xml)
{
  ci::XmlTree nn("CTRNN","");
  nn.setAttribute("Size", size);
  nn.setAttribute("Weight", getWeightSum());
  
  for(int i = 0; i < size; ++i)
  {
    ci::XmlTree n("Neuron","");
    n.setAttribute("Index", i);
    n.setAttribute("Bias", biases[i]);
    n.setAttribute("TimeConstant", taus[i]);
    n.setAttribute("Gain", gains[i]);
    
    for(int j = 0; j < size; ++j)
    {
      ci::XmlTree w("Weight", toString(weights[j][i]));
      w.setAttribute("From", j);
      n.push_back(w);
    }

    nn.push_back(n);
  }
  
  xml.push_back(nn);
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNN::fromXml(const ci::XmlTree& xml)
{
  if(xml.getTag() == "CTRNN")
  {
    assert(xml.getAttributeValue<int>("Size") == size);
    
    ci::XmlTree::ConstIter neuron = xml.begin("Neuron");
    for (; neuron != xml.end(); ++neuron)
    {
      const ci::XmlTree& n = *neuron;
      int i = n.getAttributeValue<int>("Index");
      biases[i] = n.getAttributeValue<double>("Bias");
      taus[i] = n.getAttributeValue<double>("TimeConstant");
      gains[i] = n.getAttributeValue<double>("Gain");
      
      ci::XmlTree::ConstIter weight = n.begin("Weight");
      for (; weight != n.end(); ++weight)
      {
        int from = weight->getAttributeValue<int>("From");
        double w = weight->getValue<double>();
        weights[from][i] = w;
      }
    }    
  }
}
  
//----------------------------------------------------------------------------------------------------------------------    
void CTRNN::record(Recorder& recorder)
{
  for(int i = 0; i < size; ++i)
  {
    recorder.push_back("NeuralInput" + toString(i), inputs[i]);
    recorder.push_back("NeuralState" + toString(i), states[i]);
    recorder.push_back("NeuralOutput" + toString(i), outputs[i]);
    recorder.push_back("NeuralExtInput" + toString(i), externalinputs[i]);
  }
}
  
} // namespace
		
