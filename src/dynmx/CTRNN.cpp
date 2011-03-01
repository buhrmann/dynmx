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

// helpers
//----------------------------------------------------------------------------------------------------------------------
int CTRNN::getNumRequiredParams(int N, bool numInputs)
{
  // as many gains as inputs, n*n weights, and n*2 for bias, tau
  return numInputs + (N * N) + (2 * N);
}

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
  externalinputs = new double[size];
  
  weights = new double*[size];
  for(int i = 0; i < size; i++)
  {
    weights[i] = new double[size];
    for(int j = 0; j < size; j++)
      weights[i][j] = 0.0;
  }
  
  // initialize
  std::fill(states, states + size, 0.0);
  std::fill(outputs, outputs + size, 0.0);
  std::fill(biases, biases + size, 0.0);
  std::fill(externalinputs, externalinputs + size, 0.0);  
  std::fill(gains, gains + size, 1.0);
  std::fill(taus, taus + size, 1.0);
  std::fill(Rtaus, Rtaus + size, 1.0);
}


// The destructor
//----------------------------------------------------------------------------------------------------------------------
CTRNN::~CTRNN()
{
  //TODO
}

// Set all parameters, e.g. from a genome, assumes params in [0,1]
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::decodeGenome(const double* params, int numInputs)
{
  // inputs gains
  for(int i = 0; i < size; i++)
  {
    if(i < numInputs)
      gains[i] = 10.0 * params[i];
    else
      gains[i] = 1.0;
  }
  
  // biases and taus
  int I = numInputs;
  for(int i = 0; i < size; i++)
  {
    setBias        (i, -10.0 + 20.0 * params[I + 2 * i]);
    setTimeConstant(i, 0.2 + 2.0 * params[I + 2 * i + 1]);
  }
  
  // weights
  I += 2 * size;
  int id = 0;
  for(int i = 0; i < size; i++)
  {
    for(int j = 0; j < size; j++)
    {
      setWeight(i, j, -10.0 + 20.0 * params[I + id]);
      id++;
    }
  }
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


// Integrate a circuit one step using Euler integration.
//----------------------------------------------------------------------------------------------------------------------
void CTRNN::update(double stepsize)
{
  // Update the state of all neurons.
  for (int i = 0; i < size; i++) 
  {
    double input = gains[i] * externalinputs[i];
    for (int j = 0; j < size; j++) 
    {
      input += weights[j][i] * outputs[j];
    }
    states[i] += stepsize * Rtaus[i] * (input - states[i]);
  }
  
  // Update the outputs of all neurons.
  for (int i = 0; i < size; i++)
  {
    outputs[i] = sigmoid(states[i] + biases[i]);
  }
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


#include <iomanip>
//----------------------------------------------------------------------------------------------------------------------
ostream& operator<<(ostream& os, CTRNN& c)
{
	// Set the precision
	os << setprecision(32);
	// Write the size
	os << c.size << endl << endl;
	// Write the time constants
	for (int i = 0; i < c.size; i++)
		os << c.taus[i] << " ";
	os << endl << endl;
	// Write the biases
	for (int i = 0; i < c.size; i++)
		os << c.biases[i] << " ";
	os << endl << endl;
	// Write the gains
	for (int i = 0; i < c.size; i++)
		os << c.gains[i] << " ";
	os << endl << endl;
	// Write the weights
	for (int i = 0; i < c.size; i++) {
		for (int j = 0; j < c.size; j++)
			os << c.weights[i][j] << " ";
		os << endl;
	}
	// Return the ostream
	return os;
}

//----------------------------------------------------------------------------------------------------------------------
istream& operator>>(istream& is, CTRNN& c)
{
	// Read the size
	int size;
	is >> size;
	
	// Read the time constants
	for (int i = 0; i < size; i++) 
  {
		is >> c.taus[i];
		c.Rtaus[i] = 1 / c.taus[i];
	}
	// Read the biases
	for (int i = 0; i < size; i++)
		is >> c.biases[i];
	// Read the gains
	for (int i = 0; i < size; i++)
		is >> c.gains[i];
	// Read the weights
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			is >> c.weights[i][j];
	// Return the istream		
	return is;
}		
		
