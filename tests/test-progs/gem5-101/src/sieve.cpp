// C++ program to print all primes smaller than or equal to
// n using Sieve of Eratosthenes
#include <bits/stdc++.h>
using namespace std;

int SieveOfEratosthenes(int n)
{
	// Create a boolean array "prime[0..n]" and initialize
	// all entries it as true. A value in prime[i] will
	// finally be false if i is Not a prime, else true.
	bool *prime = new bool[n+1];
	memset(prime, true, n+1);

	for (int p = 2; p * p <= n; p++) {
		// If prime[p] is not changed, then it is a prime
		if (prime[p] == true) {
			// Update all multiples of p greater than or
			// equal to the square of it numbers which are
			// multiple of p and are less than p^2 are
			// already been marked.
			for (int i = p * p; i <= n; i += p)
				prime[i] = false;
		}
	}

	int count(0);
	// Print all prime numbers
	for (int p = 2; p <= n; p++)
		if (prime[p])
			count++;
			//cout << p << " ";

	return count;
}

// Driver Code
int main()
{
	int n = 1000000;
	int count;
	//cout << "Following are the prime numbers smaller "
		//<< " than or equal to " << n << endl;
	count = SieveOfEratosthenes(n);
	cout << "The number of prime numbers smaller than or equal to "
		<< n << " is " << count << endl;
	return 0;
}

