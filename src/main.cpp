#include <gng_sam.hpp>

int main() {
	gng::GNG<2>* GrowingNeuralGas = new gng::GNG<2>();
	GrowingNeuralGas->start(10);
}
