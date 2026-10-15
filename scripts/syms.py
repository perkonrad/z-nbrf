import numpy as np, sys
d=np.loadtxt(sys.argv[1] if len(sys.argv)>1 else 'build/run/syms.txt')
y=d[:,0]+1j*d[:,1]
n=len(y); print("nsym",n)
for a,b in [(0,200),(200,600),(600,1200),(n-600,n)]:
    s=y[a:b]
    # QPSK-normalized EVM vs nearest QPSK
    dd=8192*(np.sign(s.real)+1j*np.sign(s.imag))
    e=s-dd
    print(f"[{a}:{b}] mean|y|={np.mean(abs(s)):.0f} rms|y|={np.sqrt(np.mean(abs(s)**2)):.0f} evm_qpsk={10*np.log10(np.mean(abs(e)**2)/(2*8192**2)):.1f}dB  mean|re|={np.mean(abs(s.real)):.0f} std|re|={np.std(abs(s.real)):.0f}")
