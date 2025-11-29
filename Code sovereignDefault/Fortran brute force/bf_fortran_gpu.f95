
! Fortran 2008 translation of the provided MATLAB program (VFI with default)
! Compile:  nvfortran -acc -fast -O3 -Minfo=accel -gpu=managed -gpu=cc86 -o bf200_gpu bf_fortran_gpu.f95
! Run:      ./bf200_gpu       ./bf400_gpu

module kinds
  implicit none
  integer, parameter :: dp = selected_real_kind(15, 307)
end module kinds

module utils
  use kinds
  implicit none
contains

  pure real(dp) function normal_cdf(x) result(c)
    ! Normal CDF via complementary error function: 0.5*erfc(-x/sqrt(2))
    real(dp), intent(in) :: x
    c = 0.5_dp * erfc(-x / sqrt(2.0_dp))
  end function normal_cdf

  subroutine tauchen(N, mu, rho, sigma, m, Z, Zprob)
    ! Fortran port of your MATLAB tauchen() function.
    use kinds
    implicit none
    integer,               intent(in)  :: N
    real(dp),              intent(in)  :: mu, rho, sigma, m
    real(dp), allocatable, intent(out) :: Z(:)
    real(dp), allocatable, intent(out) :: Zprob(:,:)

    integer  :: i, j, k
    real(dp) :: a, zstep, tol
    real(dp) :: zN, z1
    real(dp) :: arg1, arg2
    allocate(Z(N), Zprob(N,N))
    Z     = 0.0_dp
    Zprob = 0.0_dp

    tol = 1.0e-13_dp
    a   = (1.0_dp - rho) * mu

    zN = m * sqrt(sigma**2 / (1.0_dp - rho**2))
    z1 = -zN
    zstep = (zN - z1) / real(N-1, dp)

    Z(1)  = z1
    Z(N)  = zN
    do i = 2, N-1
      Z(i) = z1 + zstep * real(i-1, dp)
    end do

    ! shift by unconditional mean
    Z = Z + a / (1.0_dp - rho)

    do j = 1, N
      do k = 1, N
        if (k == 1) then
          arg1 = (Z(1) - a - rho*Z(j) + zstep/2.0_dp) / sigma
          Zprob(j,k) = normal_cdf(arg1)
        elseif (k == N) then
          arg1 = (Z(N) - a - rho*Z(j) - zstep/2.0_dp) / sigma
          Zprob(j,k) = 1.0_dp - normal_cdf(arg1)
        else
          arg1 = (Z(k)   - a - rho*Z(j) + zstep/2.0_dp) / sigma
          arg2 = (Z(k)   - a - rho*Z(j) - zstep/2.0_dp) / sigma
          Zprob(j,k) = normal_cdf(arg1) - normal_cdf(arg2)
        end if
      end do
    end do

    ! clean tiny negatives and row-normalize
    do i = 1, N
      do k = 1, N
        if (Zprob(i,k) < tol) Zprob(i,k) = 0.0_dp
      end do
      Zprob(i,:) = Zprob(i,:) / sum(Zprob(i,:))
    end do
  end subroutine tauchen

  subroutine kron(A, B, C)
    ! Kronecker product: C = kron(A,B)
    use kinds
    implicit none
    real(dp), intent(in)  :: A(:,:), B(:,:)
    real(dp), allocatable, intent(out) :: C(:,:)
    integer :: m, n, p, q, i, j
    m = size(A,1); n = size(A,2)
    p = size(B,1); q = size(B,2)
    allocate(C(m*p, n*q))
    do i = 1, m
      do j = 1, n
        C( (i-1)*p+1:i*p, (j-1)*q+1:j*q ) = A(i,j) * B
      end do
    end do
  end subroutine kron

  pure real(dp) function maxabsdiff(A, B) result(mx)
    ! max_{i} |A_i - B_i|
    use kinds
    implicit none
    real(dp), intent(in) :: A(:,:), B(:,:)
    integer :: i, j
    real(dp) :: tmp
    mx = 0.0_dp
    do i = 1, size(A,1)
      do j = 1, size(A,2)
        tmp = abs(A(i,j) - B(i,j))
        if (tmp > mx) mx = tmp
      end do
    end do
  end function maxabsdiff

  pure real(dp) function maxabsdiff_vec(a, b) result(mx)
    use kinds
    implicit none
    real(dp), intent(in) :: a(:), b(:)
    integer :: i
    real(dp) :: tmp
    mx = 0.0_dp
    do i = 1, size(a)
      tmp = abs(a(i) - b(i))
      if (tmp > mx) mx = tmp
    end do
  end function maxabsdiff_vec

end module utils

module econ_types
  use kinds
  implicit none
  type :: Params
    real(dp) :: phi0   ! 0.028
    real(dp) :: rstar  ! 0.01
    real(dp) :: theta  ! 0.0385
    real(dp) :: sigg   ! 2
    real(dp) :: betta  ! 0.85
  end type Params
end module econ_types

module solver_mod
  
  use openacc
  use omp_lib
  use kinds
  use utils
  use econ_types
  implicit none
contains

  subroutine solver_bruteforce(b, z, m, pdf, para, &
       q, bp, vp, def, totaltime, avgtime)
    ! Fortran port of your MATLAB solver_bruteforce
    real(dp), intent(in) :: b(:), z(:), m(:)
    real(dp), intent(in) :: pdf(:,:)
    type(Params), intent(in) :: para

    real(dp), allocatable, intent(out) :: q(:,:), vp(:,:)
    integer,  allocatable, intent(out) :: bp(:,:)
    logical,  allocatable, intent(out) :: def(:,:)
    real(dp), intent(out) :: totaltime, avgtime

    integer :: ns, nb, nb0, is, ib, j
    real(dp) :: theta, betta, sigg, rstar, phi0
    real(dp) :: start_t, end_t
    real(dp), allocatable :: vp1(:,:), vd(:), vd1(:), vo(:)
    real(dp), allocatable :: evp(:,:), ua(:), w(:,:)
    real(dp), allocatable :: qnew(:,:), tmpM(:,:), vtemp(:)
    real(dp) :: diff, tol, c1, uval, val, bestval
    integer  :: bestj, its

    theta = para%theta
    betta = para%betta
    sigg  = para%sigg
    rstar = para%rstar
    phi0  = para%phi0

    ns = size(z)
    nb = size(b)

    allocate(q(ns,nb), vp(ns,nb), vp1(ns,nb))
    allocate(vd(ns), vd1(ns), vo(ns))
    allocate(def(ns,nb), bp(ns,nb))
    allocate(evp(ns,nb), ua(ns), w(ns,nb))
    allocate(qnew(ns,nb), tmpM(ns,nb))
    allocate(vtemp(ns))

    ! find nb0 = argmin |b|
    nb0 = 1
    do j = 2, nb
      if (abs(b(j)) < abs(b(nb0))) nb0 = j
    end do

    vp  = 0.0_dp
    vd  = 0.0_dp
    vo = vd 
    def = .false.
    bp  = 0

    q = 1.0_dp / (1.0_dp + rstar)

    ! ua = ((exp(z).*m*(1-phi0))^(1-sigg) - 1)/(1-sigg)
    ua = ((exp(z) * m * (1.0_dp - phi0))**(1.0_dp - sigg) - 1.0_dp) / (1.0_dp - sigg)

    tol = 1.0e-7_dp
    diff = 1.0_dp
    its  = 1

    start_t = omp_get_wtime()

    do while (diff > tol .and. its < 1000)

      ! evp = betta * (m^(1-sigg)) .* (pdf * vp)
      tmpM = matmul(pdf, vp)            ! ns x nb
      do is = 1, ns
        evp(is,:) = betta * (m(is)**(1.0_dp - sigg)) * tmpM(is,:)
      end do

      ! vd1 = ua + betta * m^(1-sigg) .* (pdf * (theta*vo + (1-theta)*vd))
      vtemp = theta*vo + (1.0_dp - theta)*vd
      do is = 1, ns
        vd1(is) = ua(is) + betta * (m(is)**(1.0_dp - sigg)) * sum(pdf(is,:)*vtemp(:))
      end do

      ! w = b' .* q + exp(z).*m
      do is = 1, ns
        do j = 1, nb
          w(is,j) = b(j)*q(is,j) + exp(z(is))*m(is)
        end do
      end do

!$acc kernels
do is = 1, ns
  do ib = 1, nb
    if (.not. def(is,ib)) then
      bestval = -huge(1.0_dp)
      bestj   = 1
      do j = 1, nb
        c1 = w(is,j) - b(ib)
        if (c1 <= 0.0_dp) then
          uval = -huge(1.0_dp)
        else
          uval = ((c1**(1.0_dp - sigg)) - 1.0_dp) / (1.0_dp - sigg)
        end if
        val = uval + evp(is,j)
        if (val > bestval) then
          bestval = val
          bestj   = j
        end if
      end do
      vp1(is,ib) = bestval
      bp (is,ib) = bestj
    end if
  end do
end do
!$acc end kernels

      ! Default decision
      do is = 1, ns
        do ib = 1, nb
          if (def(is,ib)) then
            vp1(is,ib) = vd1(is)
            bp (is,ib) = 0
          else
            if (vp1(is,ib) > vd1(is)) then
              def(is,ib) = .false.
            else
              def(is,ib) = .true.
              vp1(is,ib) = vd1(is)
              bp (is,ib) = 0
            end if
          end if
        end do
      end do

      ! qnew = (1 - pdf * def) / (1+rstar)
      ! Cast logical->real
      tmpM = 0.0_dp
      do j = 1, nb
        tmpM(:,j) = matmul(pdf, merge(1.0_dp, 0.0_dp, def(:,j)))
      end do
      qnew = (1.0_dp - tmpM) / (1.0_dp + rstar)

      ! Convergence metric
      diff = max( maxabsdiff(qnew, q), maxabsdiff(vp1, vp) )
      diff = max( diff, maxabsdiff_vec(vd1, vd) )

      vo = vp1(:, nb0)
      vp = vp1
      vd = vd1
      q  = qnew

      if (mod(its, 50) == 0) then
        write(*,'(I6,2X,A,1X,ES12.5)') its, '~ diff', diff
      end if
      its = its + 1
    end do

    end_t = omp_get_wtime()
    totaltime = end_t - start_t
    avgtime   = totaltime / max(1, its)
    write(*,'(A,F12.6)') 'Total time: ', totaltime
    write(*,'(A,F12.6)') 'Average time per iteration: ', avgtime
    write(*,'(A,I6)')    'Number of iterations: ', its

  end subroutine solver_bruteforce

end module solver_mod

program main
      
  use kinds
  use utils
  use econ_types
  use solver_mod
  implicit none

  ! ==== Parameters (mirroring MATLAB) ====
  integer :: nb, nz, nm, ns
  real(dp) :: rhoz, sdz, width, muz
  real(dp) :: rhom, sdm, mum
  type(Params) :: para

  ! Grids & matrices
  real(dp), allocatable :: zgrid(:), pdfz(:,:)
  real(dp), allocatable :: mgrid(:), pdfm(:,:)
  real(dp), allocatable :: pdf(:,:)
  real(dp), allocatable :: z(:), m(:)
  real(dp), allocatable :: b(:)

  ! Outputs
  real(dp), allocatable :: q(:,:), vp(:,:)
  integer,  allocatable :: bp(:,:)
  logical,  allocatable :: def(:,:)
  real(dp) :: totaltime, avgtime

  integer :: iz, im, is, i
  real(dp) :: bupper, blower, step

  ! ==== Set scalars ====
  nb = 200
  para%phi0  = 0.028_dp
  para%rstar = 0.01_dp
  para%theta = 0.0385_dp
  para%sigg  = 2.0_dp
  para%betta = 0.85_dp

  nz = 25
  nm = 25

  rhoz = 0.90_dp
  sdz  = 0.030_dp
  width= 4.0_dp
  muz  = 0.0_dp

  rhom = 0.90_dp
  sdm  = 0.008_dp
  mum  = 1.02_dp

  ! ==== Tauchen for z and m ====
  call tauchen(nz, muz, rhoz, sdz, width, zgrid, pdfz)
  call tauchen(nm, mum, rhom, sdm, width, mgrid, pdfm)

  ! ==== Joint Markov via Kronecker ====
  ns = nz * nm
  call kron(pdfm, pdfz, pdf)   ! ns x ns

  ! Normalize rows (should already be row-stochastic; this mirrors MATLAB)
  do is = 1, ns
    pdf(is,:) = pdf(is,:) / sum(pdf(is,:))
  end do

  ! ==== Build stacked state arrays z,m with MATLAB order: is = (im-1)*nz + iz ====
  allocate(z(ns), m(ns))
  do im = 1, nm
    do iz = 1, nz
      is = (im-1)*nz + iz
      z(is) = zgrid(iz)
      m(is) = mgrid(im)
    end do
  end do

  ! ==== Debt grid ====
  blower = 0.0_dp
  bupper = 1.01_dp
  allocate(b(nb))
  if (nb == 1) then
    b(1) = blower
  else
    step = (bupper - blower) / real(nb-1, dp)
    b    = [( blower + step*real(i-1,dp), i=1,nb )]
  end if

  ! ==== Solve ====
  call solver_bruteforce(b, z, m, pdf, para, q, bp, vp, def, totaltime, avgtime)

  ! A simple final print to show shapes match expectation
  write(*,*) 'Done. Shapes: q(ns,nb)=', size(q,1), size(q,2), ' vp(ns,nb)=', size(vp,1), size(vp,2)
end program main
