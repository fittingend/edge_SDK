//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EDGE_fusion_function_231017_2135.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 17-Oct-2023 22:24:52
//

// Include Files
#include "EDGE_fusion_function_231017_2135.h"
#include "EDGE_fusion_function_231017_2135_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Type Definitions
struct cell_wrap_3
{
  unsigned int f1[8];
};

struct struct_T
{
  unsigned int TrackID;
  unsigned int BranchID;
  unsigned int SourceIndex;
  double UpdateTime;
  unsigned int Age;
  double State[6];
  double StateCovariance[36];
  double ObjectClassID;
  double ObjectClassProbabilities;
  char TrackLogic[7];
  boolean_T TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

struct b_struct_T
{
  unsigned int TrackID;
  unsigned int BranchID;
  unsigned int SourceIndex;
  double UpdateTime;
  unsigned int Age;
  double State[6];
  double StateCovariance[36];
  double ObjectClassID;
  double ObjectClassProbabilities;
  char TrackLogic[7];
  boolean_T TrackLogicState[5];
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

namespace coder
{
  class objectTrack
  {
   public:
    unsigned int TrackID;
    unsigned int BranchID;
    unsigned int SourceIndex;
    unsigned int Age;
    double ObjectClassID;
    double ObjectClassProbabilities;
    boolean_T IsConfirmed;
    boolean_T IsCoasted;
    boolean_T IsSelfReported;
    double pState[6];
    double pStateCovariance[36];
    double pUpdateTime;
    array<boolean_T, 2U> pTrackLogicState;
  };

  class trackHistoryLogic
  {
   public:
    boolean_T pRecentHistory[50];
    boolean_T pIsFirstUpdate;
  };

  class fuserSourceConfiguration
  {
   public:
    fuserSourceConfiguration *clone(fuserSourceConfiguration &iobj_0) const;
    double SourceIndex;
    boolean_T IsInternalSource;
    boolean_T IsInitializingCentralTracks;
    boolean_T pIsTransformToCentralValid;
    boolean_T pIsTransformToLocalValid;
  };

  namespace fusion
  {
    namespace internal
    {
      class Fuserxcov
      {
       public:
        void fuse(objectTrack &centralTrack, const array<objectTrack, 2U>
                  &sourceTracks, const array<double, 1U> &inAssigned) const;
        double ProcessNoise[9];
      };
    }
  }

  namespace matlabshared
  {
    namespace tracking
    {
      namespace internal
      {
        namespace fusion
        {
          class AssignerGNN
          {
           public:
            int isInitialized;
            boolean_T isSetupComplete;
          };
        }
      }
    }
  }

  class trackFuser
  {
   public:
    int step(const array<struct_T, 2U> &varargin_1, double varargin_2, struct3_T
             varargout_1_data[], struct3_T varargout_2_data[], array<struct3_T,
             1U> &varargout_3, unsigned int
             varargout_4_TrackIDsAtStepBeginning_data[], int
             varargout_4_TrackIDsAtStepBeginning_size[2], array<double, 2U>
             &varargout_4_CostMatrix, array<unsigned int, 2U>
             &varargout_4_Assignments, array<unsigned int, 1U>
             &varargout_4_UnassignedCentralTracks, array<unsigned int, 1U>
             &varargout_4_UnassignedLocalTracks, array<unsigned int, 1U>
             &varargout_4_NonInitializingLocalTracks, unsigned int
             varargout_4_InitializedCentralTrackIDs_data[], int
             varargout_4_InitializedCentralTrackIDs_size[2], array<unsigned int,
             2U> &varargout_4_UpdatedCentralTrackIDs, unsigned int
             varargout_4_DeletedTrackIDs_data[], int
             varargout_4_DeletedTrackIDs_size[2], unsigned int
             varargout_4_TrackIDsAtStepEnd_data[], int
             varargout_4_TrackIDsAtStepEnd_size[2], int &varargout_2_size);
    void matlabCodegenDestructor();
    void releaseWrapper();
    ~trackFuser();
    trackFuser();
   protected:
    void setupImpl(const array<struct_T, 2U> &tracks);
    void resetImpl();
    int stepImpl(const array<struct_T, 2U> &localTracks, double tFusion,
                 struct3_T confTracks_data[], struct3_T tentTracks_data[], array<
                 struct3_T, 1U> &allTracks, unsigned int
                 info_TrackIDsAtStepBeginning_data[], int
                 info_TrackIDsAtStepBeginning_size[2], array<double, 2U>
                 &info_CostMatrix, array<unsigned int, 2U> &info_Assignments,
                 array<unsigned int, 1U> &info_UnassignedCentralTracks, array<
                 unsigned int, 1U> &info_UnassignedLocalTracks, array<unsigned
                 int, 1U> &info_NonInitializingLocalTracks, unsigned int
                 info_InitializedCentralTrackIDs_data[], int
                 info_InitializedCentralTrackIDs_size[2], array<unsigned int, 2U>
                 &info_UpdatedCentralTrackIDs, unsigned int
                 info_DeletedTrackIDs_data[], int info_DeletedTrackIDs_size[2],
                 unsigned int info_TrackIDsAtStepEnd_data[], int
                 info_TrackIDsAtStepEnd_size[2], int &tentTracks_size);
    void assign(const array<double, 2U> &costMatrix, array<unsigned int, 2U>
                &overallAssignments, array<unsigned int, 1U>
                &overallUnassignedCentralTracks, array<unsigned int, 1U>
                &overallUnassignedLocalTracks);
    void initializeCentralTracks(const array<struct_T, 2U> &localTracks, array<
      unsigned int, 1U> &unassignedlocalTracks);
    static void ensureTrack(unsigned int in_TrackID, unsigned int in_SourceIndex,
      double in_UpdateTime, const double in_State[6], const double
      in_StateCovariance[36], double in_ObjectClassID, const char in_TrackLogic
      [7], objectTrack *out);
    void fuseAssigned(const array<struct_T, 2U> &localTracks, const array<
                      unsigned int, 2U> &assignments);
    boolean_T getSelfReporting(const array<struct_T, 2U> &localTracks, const
      array<unsigned int, 1U> &assignedlocalTracks);
    void b_fuseAssigned(const array<struct_T, 2U> &localTracks, const array<
                        unsigned int, 2U> &assignments, array<boolean_T, 2U>
                        &updated);
    void coastUnassigned(const array<unsigned int, 1U> &unassignedTracks, const
                         array<double, 2U> &notUpdated, boolean_T toDelete[100]);
    void distance(const array<struct_T, 2U> &localTracks, array<double, 2U>
                  &costMatrix);
   public:
    boolean_T matlabCodegenIsDeleted;
    int isInitialized;
    boolean_T TunablePropsChanged;
    double ProcessNoise[9];
    objectTrack pTracksList[100];
    trackHistoryLogic *pTrackLogics[100];
    double pSourceConfigIDs[20];
    fuserSourceConfiguration *pSourceConfigurations[20];
    double pNumUsedConfigs;
    boolean_T pIsValidSource[20];
    matlabshared::tracking::internal::fusion::AssignerGNN cAssigner;
    trackHistoryLogic _pobj0[100];
    fuserSourceConfiguration _pobj1[20];
   protected:
    double pNumLiveTracks;
    unsigned int pTrackIDs[100];
    boolean_T pConfirmedTracks[100];
    array<boolean_T, 2U> pUsedConfigIDs;
    double pLastTimeStamp;
    unsigned int pLastTrackID;
   private:
    boolean_T isSetupComplete;
    cell_wrap_3 inputVarSize[2];
    fusion::internal::Fuserxcov cFuser;
  };

  namespace matlab
  {
    namespace internal
    {
      namespace coder
      {
        class minPriorityQueue
        {
         public:
          void percUp(int i, const array<double, 1U> &dist);
          array<int, 1U> heap;
          array<int, 1U> indexToHeap;
          int len;
        };
      }
    }
  }

  namespace fusion
  {
    namespace internal
    {
      class gaussEKFilter
      {
       public:
        static void predict(const double x[6], double P[36], const double Q[9],
                            double varargin_1, double xk[6]);
      };
    }
  }
}

// Variable Definitions
static const signed char iv[36]{ 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

// Function Declarations
static void binary_expand_op(coder::array<double, 2U> &in1, int in2, const coder::
  array<double, 2U> &in3);
static void binary_expand_op_1(coder::array<double, 2U> &in1, int in2, int in3,
  const coder::array<double, 2U> &in4, const coder::array<double, 2U> &in5);
namespace coder
{
  static void do_vectors(const array<unsigned int, 1U> &a, const array<unsigned
    int, 1U> &b, array<unsigned int, 1U> &c, array<int, 1U> &ia, array<int, 1U>
    &ib);
  static void eigStandard(const double A[36], creal_T V[36], creal_T D[36]);
  static void eml_find(const array<boolean_T, 1U> &x, array<int, 1U> &i);
  namespace fusion
  {
    namespace internal
    {
      static void ensurePosDefMatrix(double P[36]);
    }
  }

  namespace internal
  {
    namespace blas
    {
      static void xaxpy(int n, double a, const double x[36], int ix0, double y
                        [18]);
      static void xgemv(int n, const double x[18], double beta1, double y[36],
                        int iy0);
      static double xnrm2(int n, const double x[36], int ix0);
      static double xnrm2(int n, const double x[3]);
      static double xnrm2(int n, const array<double, 2U> &x, int ix0);
    }

    static void merge(array<int, 1U> &idx, array<unsigned int, 1U> &x, int
                      offset, int np, int nq, array<int, 1U> &iwork, array<
                      unsigned int, 1U> &xwork);
    static void merge(array<int, 2U> &idx, array<double, 2U> &x, int offset, int
                      np, int nq, array<int, 1U> &iwork, array<double, 1U>
                      &xwork);
    static void merge_block(array<int, 1U> &idx, array<unsigned int, 1U> &x, int
      offset, int n, int preSortLevel, array<int, 1U> &iwork, array<unsigned int,
      1U> &xwork);
    static void merge_block(array<int, 2U> &idx, array<double, 2U> &x, int
      offset, int n, int preSortLevel, array<int, 1U> &iwork, array<double, 1U>
      &xwork);
    static void mrdiv(double A[6], const double B[36]);
    static void qrsolve(const array<double, 2U> &A, const array<double, 1U> &B,
                        array<double, 1U> &Y);
    namespace reflapack
    {
      static void b_rotateRight(int n, double z[36], int iz0, const double cs[10],
        int ic0, int is0);
      static void b_xzlascl(double cfrom, double cto, int m, double A[5], int
                            iA0);
      static void rotateRight(int n, double z[36], int iz0, const double cs[10],
        int ic0, int is0);
      static double xdladiv(double a, double b, double c, double d, double &q);
      static double xdlaev2(double a, double b, double c, double &rt2, double
                            &cs1, double &sn1);
      static int xdlahqr(int ilo, int ihi, double h[36], int iloz, int ihiz,
                         double z[36], double wr[6], double wi[6]);
      static double xdlaln2(int na, int nw, double smin, const double A[36], int
                            ia0, const double B[18], int ib0, double wr, double
                            wi, double X[4], double &xnorm);
      static double xdlanv2(double &a, double &b, double &c, double &d, double
                            &rt1i, double &rt2r, double &rt2i, double &cs,
                            double &sn);
      static void xdtrevc3(const double T[36], double vr[36]);
      static int xzgebal(double A[36], int &ihi, double scale[6]);
      static void xzgehrd(double a[36], int ilo, int ihi, double tau[5]);
      static int xzgetrf(double A[36], int ipiv[6]);
      static void xzlarf(int m, int n, int iv0, double tau, double C[36], int
                         ic0, double work[6]);
      static double xzlarfg(int n, double &alpha1, double x[36], int ix0);
      static double xzlarfg(int n, double &alpha1, double x[3]);
      static double xzlartg(double f, double g, double &sn, double &r);
      static void xzlascl(double cfrom, double cto, int m, double A[6], int iA0);
      static void xzlascl(double cfrom, double cto, double A[36]);
      static int xzsteqr(double d[6], double e[5], double z[36]);
      static void xzunghr(int ilo, int ihi, double A[36], const double tau[5]);
    }

    static void sort(array<double, 2U> &x, array<int, 2U> &idx);
    static void sort(array<unsigned int, 1U> &x);
  }

  static void mldivide(const double A[36], double Y[36]);
  static void perfectMatching(const array<double, 2U> &A, array<int, 1U> &m1,
    array<int, 1U> &m2);
  static void unique_vector(const array<double, 2U> &a, array<double, 2U> &b);
  static void unique_vector(const array<unsigned int, 1U> &a, array<unsigned int,
    1U> &b);
}

static int div_nde_s32_floor(int numerator);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : const array<double, 2U> &costMatrix
//                array<unsigned int, 2U> &overallAssignments
//                array<unsigned int, 1U> &overallUnassignedCentralTracks
//                array<unsigned int, 1U> &overallUnassignedLocalTracks
// Return Type  : void
//
namespace coder
{
  void trackFuser::assign(const array<double, 2U> &costMatrix, array<unsigned
    int, 2U> &overallAssignments, array<unsigned int, 1U>
    &overallUnassignedCentralTracks, array<unsigned int, 1U>
    &overallUnassignedLocalTracks)
  {
    array<double, 2U> b_paddedCost;
    array<double, 2U> paddedCost;
    array<double, 2U> y;
    array<unsigned int, 2U> assignments;
    array<int, 2U> matchings;
    array<int, 1U> b_r;
    array<int, 1U> colToRow;
    array<int, 1U> r1;
    array<int, 1U> rowToCol;
    array<int, 1U> sourceIndex;
    array<unsigned int, 1U> unassignedCentralTracks;
    array<int, 1U> unassignedCols;
    array<int, 1U> unassignedRows;
    array<unsigned int, 1U> unassignedlocalTracks;
    array<boolean_T, 1U> obj;
    double lastAssigned;
    double lastUnassigned;
    double numCentralTrks;
    int eint;
    int i;
    int i1;
    int i2;
    int ix;
    int j;
    int n;
    int nz;
    int r;
    signed char tmp_data[20];
    boolean_T usedSources[20];
    numCentralTrks = pNumLiveTracks;
    for (i = 0; i < 20; i++) {
      usedSources[i] = false;
    }

    j = pUsedConfigIDs.size(0);
    i2 = 1;
    for (n = 0; n < 20; n++) {
      boolean_T exitg1;
      r = (i2 + j) - 1;
      ix = i2;
      i2 += pUsedConfigIDs.size(0);
      exitg1 = false;
      while ((!exitg1) && (ix <= r)) {
        if (pUsedConfigIDs[ix - 1]) {
          usedSources[n] = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    nz = usedSources[0];
    for (i2 = 0; i2 < 19; i2++) {
      nz += usedSources[i2 + 1];
    }

    i = static_cast<int>(numCentralTrks * static_cast<double>(nz));
    overallAssignments.set_size(i, 2);
    j = i << 1;
    for (i = 0; i < j; i++) {
      overallAssignments[i] = 0U;
    }

    overallUnassignedLocalTracks.set_size(costMatrix.size(1));
    j = costMatrix.size(1);
    for (i = 0; i < j; i++) {
      overallUnassignedLocalTracks[i] = 0U;
    }

    if (std::isnan(numCentralTrks)) {
      y.set_size(1, 1);
      y[0] = rtNaN;
    } else if (numCentralTrks < 1.0) {
      y.set_size(1, 0);
    } else {
      y.set_size(1, static_cast<int>(numCentralTrks - 1.0) + 1);
      j = static_cast<int>(numCentralTrks - 1.0);
      for (i = 0; i <= j; i++) {
        y[i] = static_cast<double>(i) + 1.0;
      }
    }

    overallUnassignedCentralTracks.set_size(y.size(1));
    j = y.size(1);
    for (i = 0; i < j; i++) {
      unsigned int u;
      numCentralTrks = y[i];
      if (numCentralTrks < 4.294967296E+9) {
        u = static_cast<unsigned int>(numCentralTrks);
      } else {
        u = 0U;
      }

      overallUnassignedCentralTracks[i] = u;
    }

    lastAssigned = 0.0;
    lastUnassigned = 0.0;
    for (int s{0}; s < nz; s++) {
      j = 0;
      for (n = 0; n < 20; n++) {
        if (usedSources[n]) {
          tmp_data[j] = static_cast<signed char>(n);
          j++;
        }
      }

      i = tmp_data[s];
      j = pUsedConfigIDs.size(0);
      obj.set_size(j);
      for (i1 = 0; i1 < j; i1++) {
        obj[i1] = pUsedConfigIDs[i1 + pUsedConfigIDs.size(0) * i];
      }

      eml_find(obj, rowToCol);
      sourceIndex.set_size(rowToCol.size(0));
      j = rowToCol.size(0);
      for (i = 0; i < j; i++) {
        sourceIndex[i] = rowToCol[i];
      }

      if (cAssigner.isInitialized != 1) {
        cAssigner.isInitialized = 1;
        cAssigner.isSetupComplete = true;
      }

      i = costMatrix.size(0);
      n = sourceIndex.size(0) - 1;
      i2 = costMatrix.size(0) + sourceIndex.size(0);
      paddedCost.set_size(i2, i2);
      ix = i2 * i2;
      for (i1 = 0; i1 < ix; i1++) {
        paddedCost[i1] = rtInf;
      }

      for (j = 0; j <= n; j++) {
        for (r = 0; r < i; r++) {
          paddedCost[r + paddedCost.size(0) * j] = costMatrix[r +
            costMatrix.size(0) * (sourceIndex[j] - 1)];
        }
      }

      for (j = 0; j < i; j++) {
        for (r = 0; r <= n; r++) {
          paddedCost[(i + r) + paddedCost.size(0) * ((n + j) + 1)] =
            costMatrix[j + costMatrix.size(0) * (sourceIndex[r] - 1)];
        }
      }

      for (r = 0; r < i; r++) {
        paddedCost[r + paddedCost.size(0) * ((n + r) + 1)] = 5.0;
      }

      for (j = 0; j <= n; j++) {
        paddedCost[(i + j) + paddedCost.size(0) * j] = 5.0;
      }

      perfectMatching(paddedCost, colToRow, rowToCol);
      if ((colToRow.size(0) == 0) && (i2 > 0)) {
        double maxVal;
        maxVal = 0.0;
        for (i2 = 0; i2 < ix; i2++) {
          numCentralTrks = std::abs(paddedCost[i2]);
          if ((numCentralTrks > maxVal) && (!std::isinf(numCentralTrks))) {
            maxVal = numCentralTrks;
          }
        }

        if (!std::isinf(maxVal)) {
          numCentralTrks = std::frexp(maxVal, &eint);
          maxVal = eint;
          if (numCentralTrks == 0.5) {
            maxVal = static_cast<double>(eint) - 1.0;
          }
        }

        numCentralTrks = rt_powd_snf(2.0, -maxVal);
        b_paddedCost.set_size(paddedCost.size(0), paddedCost.size(1));
        for (i1 = 0; i1 < ix; i1++) {
          b_paddedCost[i1] = paddedCost[i1] * numCentralTrks;
        }

        perfectMatching(b_paddedCost, colToRow, rowToCol);
      }

      i2 = 0;
      for (j = 0; j <= n; j++) {
        i1 = colToRow[j];
        r = colToRow[j];
        if ((colToRow[j] <= i) && (costMatrix[(colToRow[j] + costMatrix.size(0) *
              (sourceIndex[j] - 1)) - 1] == 5.0)) {
          ix = i + n;
          i1 = ix + 2;
          colToRow[j] = ix + 2;
          rowToCol[r - 1] = ix + 2;
        }

        if (i1 <= i) {
          i2++;
        }
      }

      matchings.set_size(i2, 2);
      j = -1;
      for (i2 = 0; i2 <= n; i2++) {
        if (colToRow[i2] <= i) {
          j++;
          matchings[j] = colToRow[i2];
          matchings[j + matchings.size(0)] = i2 + 1;
        }
      }

      i1 = costMatrix.size(0);
      unassignedRows.set_size(costMatrix.size(0));
      i2 = 0;
      for (r = 0; r < i1; r++) {
        if (rowToCol[r] > n + 1) {
          i2++;
          unassignedRows[i2 - 1] = r + 1;
        }
      }

      if (i2 < 1) {
        i2 = 0;
      }

      unassignedRows.set_size(i2);
      i1 = sourceIndex.size(0);
      unassignedCols.set_size(sourceIndex.size(0));
      i2 = 0;
      for (r = 0; r < i1; r++) {
        if (colToRow[r] > i) {
          i2++;
          unassignedCols[i2 - 1] = r + 1;
        }
      }

      if (i2 < 1) {
        i2 = 0;
      }

      unassignedCols.set_size(i2);
      assignments.set_size(matchings.size(0), 2);
      j = matchings.size(0) << 1;
      for (i = 0; i < j; i++) {
        i1 = matchings[i];
        if (i1 < 0) {
          i1 = 0;
        }

        assignments[i] = static_cast<unsigned int>(i1);
      }

      unassignedCentralTracks.set_size(unassignedRows.size(0));
      j = unassignedRows.size(0);
      for (i = 0; i < j; i++) {
        unassignedCentralTracks[i] = static_cast<unsigned int>(unassignedRows[i]);
      }

      unassignedlocalTracks.set_size(unassignedCols.size(0));
      j = unassignedCols.size(0);
      for (i = 0; i < j; i++) {
        unassignedlocalTracks[i] = static_cast<unsigned int>(unassignedCols[i]);
      }

      if (assignments.size(0) != 0) {
        numCentralTrks = lastAssigned + static_cast<double>(assignments.size(0));
        if (lastAssigned + 1.0 > numCentralTrks) {
          i = 1;
        } else {
          i = static_cast<int>(lastAssigned + 1.0);
        }

        j = assignments.size(0);
        for (i1 = 0; i1 < j; i1++) {
          overallAssignments[(i + i1) - 1] = assignments[i1];
        }

        if (lastAssigned + 1.0 > numCentralTrks) {
          i = 1;
        } else {
          i = static_cast<int>(lastAssigned + 1.0);
        }

        j = assignments.size(0);
        for (i1 = 0; i1 < j; i1++) {
          overallAssignments[((i + i1) + overallAssignments.size(0)) - 1] =
            static_cast<unsigned int>(sourceIndex[static_cast<int>
            (assignments[i1 + assignments.size(0)]) - 1]);
        }

        lastAssigned += static_cast<double>(assignments.size(0));
      }

      if (unassignedlocalTracks.size(0) != 0) {
        if (lastUnassigned + 1.0 > lastUnassigned + static_cast<double>
            (unassignedlocalTracks.size(0))) {
          i = 1;
        } else {
          i = static_cast<int>(lastUnassigned + 1.0);
        }

        j = unassignedlocalTracks.size(0);
        for (i1 = 0; i1 < j; i1++) {
          overallUnassignedLocalTracks[(i + i1) - 1] = static_cast<unsigned int>
            (sourceIndex[static_cast<int>(unassignedlocalTracks[i1]) - 1]);
        }

        lastUnassigned += static_cast<double>(unassignedlocalTracks.size(0));
      }

      if (unassignedCentralTracks.size(0) != 0) {
        internal::sort(unassignedCentralTracks);
        unassignedlocalTracks.set_size(overallUnassignedCentralTracks.size(0));
        j = overallUnassignedCentralTracks.size(0) - 1;
        for (i = 0; i <= j; i++) {
          unassignedlocalTracks[i] = overallUnassignedCentralTracks[i];
        }

        do_vectors(unassignedlocalTracks, unassignedCentralTracks,
                   overallUnassignedCentralTracks, rowToCol, unassignedCols);
      } else {
        overallUnassignedCentralTracks.set_size(0);
      }
    }

    i2 = overallUnassignedLocalTracks.size(0) - 1;
    j = 0;
    for (n = 0; n <= i2; n++) {
      if (static_cast<int>(overallUnassignedLocalTracks[n]) > 0) {
        j++;
      }
    }

    b_r.set_size(j);
    j = 0;
    for (n = 0; n <= i2; n++) {
      if (static_cast<int>(overallUnassignedLocalTracks[n]) > 0) {
        b_r[j] = n;
        j++;
      }
    }

    unassignedCentralTracks.set_size(b_r.size(0));
    j = b_r.size(0);
    for (i = 0; i < j; i++) {
      unassignedCentralTracks[i] = overallUnassignedLocalTracks[b_r[i]];
    }

    unique_vector(unassignedCentralTracks, overallUnassignedLocalTracks);
    obj.set_size(overallAssignments.size(0));
    j = overallAssignments.size(0);
    for (i = 0; i < j; i++) {
      obj[i] = (static_cast<int>(overallAssignments[i]) > 0);
    }

    i2 = obj.size(0) - 1;
    j = 0;
    for (n = 0; n <= i2; n++) {
      if (obj[n]) {
        j++;
      }
    }

    r1.set_size(j);
    j = 0;
    for (n = 0; n <= i2; n++) {
      if (obj[n]) {
        r1[j] = n;
        j++;
      }
    }

    assignments.set_size(r1.size(0), 2);
    j = r1.size(0);
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < j; i1++) {
        assignments[i1 + assignments.size(0) * i] = overallAssignments[r1[i1] +
          overallAssignments.size(0) * i];
      }
    }

    overallAssignments.set_size(assignments.size(0), 2);
    j = assignments.size(0) << 1;
    for (i = 0; i < j; i++) {
      overallAssignments[i] = assignments[i];
    }
  }

  //
  // Arguments    : const array<struct_T, 2U> &localTracks
  //                const array<unsigned int, 2U> &assignments
  //                array<boolean_T, 2U> &updated
  // Return Type  : void
  //
  void trackFuser::b_fuseAssigned(const array<struct_T, 2U> &localTracks, const
    array<unsigned int, 2U> &assignments, array<boolean_T, 2U> &updated)
  {
    fuserSourceConfiguration *thisConfig;
    objectTrack obj;
    trackHistoryLogic *b_obj;
    array<objectTrack, 2U> transformedTracks;
    array<double, 1U> b_assignedSourceInds;
    array<int, 1U> assignedSourceInds;
    array<unsigned int, 1U> b_assignments;
    array<int, 1U> r;
    array<int, 1U> r1;
    array<unsigned int, 1U> uniqueAssigned;
    array<boolean_T, 1U> inAssigned;
    array<boolean_T, 1U> toFuse;
    int c_i;
    int i;
    int k;
    int ntilecols;
    int nz;
    boolean_T exitg1;
    obj = pTracksList[0];
    transformedTracks.set_size(1, assignments.size(0));
    if (assignments.size(0) != 0) {
      ntilecols = assignments.size(0);
      for (nz = 0; nz < ntilecols; nz++) {
        transformedTracks[nz] = obj;
      }
    }

    i = assignments.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      struct_T expl_temp;
      boolean_T inKnownIDs[20];
      expl_temp = localTracks[static_cast<int>(assignments[b_i +
        assignments.size(0)]) - 1];
      for (k = 0; k < 20; k++) {
        inKnownIDs[k] = (expl_temp.SourceIndex == pSourceConfigIDs[k]);
      }

      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 20)) {
        if (inKnownIDs[c_i]) {
          thisConfig = pSourceConfigurations[c_i];
          exitg1 = true;
        } else {
          c_i++;
        }
      }

      if (!thisConfig->pIsTransformToCentralValid) {
        thisConfig->pIsTransformToCentralValid = true;
      }

      trackFuser::ensureTrack(expl_temp.TrackID, expl_temp.SourceIndex,
        expl_temp.UpdateTime, expl_temp.State, expl_temp.StateCovariance,
        expl_temp.ObjectClassID, expl_temp.TrackLogic, &transformedTracks[b_i]);
    }

    b_assignments.set_size(assignments.size(0));
    ntilecols = assignments.size(0);
    for (i = 0; i < ntilecols; i++) {
      b_assignments[i] = assignments[i];
    }

    unique_vector(b_assignments, uniqueAssigned);
    updated.set_size(1, uniqueAssigned.size(0));
    ntilecols = uniqueAssigned.size(0);
    for (i = 0; i < ntilecols; i++) {
      updated[i] = false;
    }

    i = uniqueAssigned.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      boolean_T y;
      ntilecols = assignments.size(0);
      inAssigned.set_size(assignments.size(0));
      for (k = 0; k < ntilecols; k++) {
        inAssigned[k] = (uniqueAssigned[b_i] == assignments[k]);
      }

      eml_find(inAssigned, assignedSourceInds);
      toFuse.set_size(assignedSourceInds.size(0));
      k = assignedSourceInds.size(0);
      for (c_i = 0; c_i < k; c_i++) {
        if ((transformedTracks[assignedSourceInds[c_i] - 1].IsConfirmed &&
             (!transformedTracks[assignedSourceInds[c_i] - 1].IsCoasted)) ||
            transformedTracks[assignedSourceInds[c_i] - 1].IsSelfReported) {
          toFuse[c_i] = true;
        } else {
          toFuse[c_i] = false;
        }
      }

      y = false;
      ntilecols = 1;
      exitg1 = false;
      while ((!exitg1) && (ntilecols <= toFuse.size(0))) {
        if (toFuse[ntilecols - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ntilecols++;
        }
      }

      if (y) {
        double id;
        int obj_tmp_tmp;
        unsigned int u;
        ntilecols = toFuse.size(0);
        if (toFuse.size(0) == 0) {
          nz = 0;
        } else {
          nz = toFuse[0];
          for (k = 2; k <= ntilecols; k++) {
            nz += toFuse[k - 1];
          }
        }

        obj_tmp_tmp = static_cast<int>(uniqueAssigned[b_i]) - 1;
        obj = pTracksList[obj_tmp_tmp];
        id = static_cast<double>(pTracksList[obj_tmp_tmp].Age) + static_cast<
          double>(nz);
        if (id < 4.294967296E+9) {
          if (id >= 0.0) {
            u = static_cast<unsigned int>(id);
          } else {
            u = 0U;
          }
        } else {
          u = MAX_uint32_T;
        }

        obj.Age = u;
        pTracksList[obj_tmp_tmp] = obj;
        nz = inAssigned.size(0) - 1;
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (inAssigned[c_i]) {
            ntilecols++;
          }
        }

        r.set_size(ntilecols);
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (inAssigned[c_i]) {
            r[ntilecols] = c_i;
            ntilecols++;
          }
        }

        id = pTracksList[obj_tmp_tmp].ObjectClassID;
        ntilecols = 0;
        while ((id == 0.0) && (ntilecols < r.size(0))) {
          ntilecols++;
          id = localTracks[static_cast<int>(assignments[r[ntilecols - 1] +
            assignments.size(0)]) - 1].ObjectClassID;
        }

        obj = pTracksList[obj_tmp_tmp];
        obj.ObjectClassID = id;
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        nz = toFuse.size(0) - 1;
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (toFuse[c_i]) {
            ntilecols++;
          }
        }

        r1.set_size(ntilecols);
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (toFuse[c_i]) {
            r1[ntilecols] = c_i;
            ntilecols++;
          }
        }

        ntilecols = r1.size(0);
        b_assignedSourceInds.set_size(r1.size(0));
        for (k = 0; k < ntilecols; k++) {
          b_assignedSourceInds[k] = assignedSourceInds[r1[k]];
        }

        cFuser.fuse(obj, transformedTracks, b_assignedSourceInds);
        pTracksList[obj_tmp_tmp] = obj;
        ntilecols = toFuse.size(0);
        if (toFuse.size(0) == 0) {
          nz = 0;
        } else {
          nz = toFuse[0];
          for (k = 2; k <= ntilecols; k++) {
            nz += toFuse[k - 1];
          }
        }

        for (ntilecols = 0; ntilecols < nz; ntilecols++) {
          boolean_T bv[50];
          b_obj = pTrackLogics[obj_tmp_tmp];
          bv[0] = true;
          for (k = 0; k < 49; k++) {
            bv[k + 1] = b_obj->pRecentHistory[k];
          }

          for (k = 0; k < 50; k++) {
            b_obj->pRecentHistory[k] = bv[k];
          }

          b_obj->pIsFirstUpdate = false;
        }

        if (pTracksList[obj_tmp_tmp].IsConfirmed) {
          y = true;
        } else {
          b_obj = pTrackLogics[obj_tmp_tmp];
          if (b_obj->pIsFirstUpdate) {
            y = false;
          } else {
            boolean_T x[5];
            for (k = 0; k < 5; k++) {
              x[k] = b_obj->pRecentHistory[k];
            }

            y = ((((x[0] + x[1]) + x[2]) + x[3]) + x[4] >= 5);
          }

          if (y || (pTracksList[obj_tmp_tmp].ObjectClassID > 0.0)) {
            y = true;
          } else {
            y = false;
          }
        }

        obj = pTracksList[obj_tmp_tmp];
        obj.IsConfirmed = y;
        pTracksList[obj_tmp_tmp] = obj;
        pConfirmedTracks[obj_tmp_tmp] = pTracksList[obj_tmp_tmp].IsConfirmed;
        obj = pTracksList[obj_tmp_tmp];
        obj.IsCoasted = false;
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        ntilecols = r.size(0);
        b_assignments.set_size(r.size(0));
        for (k = 0; k < ntilecols; k++) {
          b_assignments[k] = assignments[r[k] + assignments.size(0)];
        }

        obj.IsSelfReported = getSelfReporting(localTracks, b_assignments);
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        pTracksList[obj_tmp_tmp] = obj;
        updated[b_i] = true;
      }
    }
  }

  //
  // Arguments    : fuserSourceConfiguration &iobj_0
  // Return Type  : fuserSourceConfiguration *
  //
  fuserSourceConfiguration *fuserSourceConfiguration::clone
    (fuserSourceConfiguration &iobj_0) const
  {
    fuserSourceConfiguration *clonedObj;
    iobj_0.pIsTransformToCentralValid = false;
    iobj_0.pIsTransformToLocalValid = false;
    clonedObj = &iobj_0;
    iobj_0.SourceIndex = SourceIndex;
    iobj_0.IsInternalSource = IsInternalSource;
    iobj_0.IsInitializingCentralTracks = IsInitializingCentralTracks;
    return clonedObj;
  }

  //
  // Arguments    : const array<unsigned int, 1U> &unassignedTracks
  //                const array<double, 2U> &notUpdated
  //                boolean_T toDelete[100]
  // Return Type  : void
  //
  void trackFuser::coastUnassigned(const array<unsigned int, 1U>
    &unassignedTracks, const array<double, 2U> &notUpdated, boolean_T toDelete
    [100])
  {
    objectTrack temp;
    trackHistoryLogic *tempLogic;
    array<unsigned int, 2U> a;
    array<unsigned int, 2U> b_unassignedTracks;
    array<int, 2U> idx;
    array<int, 1U> iwork;
    array<boolean_T, 2U> c_x;
    double d;
    int b_i;
    int i;
    int i1;
    int j;
    int k;
    int n;
    int na;
    int nb;
    int nz;
    int p;
    int pEnd;
    int qEnd;
    unsigned int u;
    unsigned int x;
    signed char ii_data[100];
    boolean_T exitg1;
    a.set_size(1, unassignedTracks.size(0) + notUpdated.size(1));
    pEnd = unassignedTracks.size(0);
    for (i = 0; i < pEnd; i++) {
      a[i] = unassignedTracks[i];
    }

    pEnd = notUpdated.size(1);
    for (i = 0; i < pEnd; i++) {
      d = std::round(notUpdated[i]);
      if (d < 4.294967296E+9) {
        if (d >= 0.0) {
          u = static_cast<unsigned int>(d);
        } else {
          u = 0U;
        }
      } else if (d >= 4.294967296E+9) {
        u = MAX_uint32_T;
      } else {
        u = 0U;
      }

      a[i + unassignedTracks.size(0)] = u;
    }

    na = a.size(1);
    n = a.size(1) + 1;
    idx.set_size(1, a.size(1));
    pEnd = a.size(1);
    for (i = 0; i < pEnd; i++) {
      idx[i] = 0;
    }

    if (a.size(1) != 0) {
      iwork.set_size(a.size(1));
      i = a.size(1) - 1;
      for (k = 1; k <= i; k += 2) {
        if (a[k - 1] <= a[k]) {
          idx[k - 1] = k;
          idx[k] = k + 1;
        } else {
          idx[k - 1] = k + 1;
          idx[k] = k;
        }
      }

      if ((a.size(1) & 1) != 0) {
        idx[a.size(1) - 1] = a.size(1);
      }

      b_i = 2;
      while (b_i < n - 1) {
        nb = b_i << 1;
        j = 1;
        for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
          int kEnd;
          p = j;
          nz = pEnd;
          qEnd = j + nb;
          if (qEnd > n) {
            qEnd = n;
          }

          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            i = idx[p - 1];
            i1 = idx[nz - 1];
            if (a[i - 1] <= a[i1 - 1]) {
              iwork[k] = i;
              p++;
              if (p == pEnd) {
                while (nz < qEnd) {
                  k++;
                  iwork[k] = idx[nz - 1];
                  nz++;
                }
              }
            } else {
              iwork[k] = i1;
              nz++;
              if (nz == qEnd) {
                while (p < pEnd) {
                  k++;
                  iwork[k] = idx[p - 1];
                  p++;
                }
              }
            }

            k++;
          }

          for (k = 0; k < kEnd; k++) {
            idx[(j + k) - 1] = iwork[k];
          }

          j = qEnd;
        }

        b_i = nb;
      }
    }

    b_unassignedTracks.set_size(1, a.size(1));
    for (k = 0; k < na; k++) {
      b_unassignedTracks[k] = a[idx[k] - 1];
    }

    nb = 0;
    k = 1;
    while (k <= na) {
      x = b_unassignedTracks[k - 1];
      do {
        k++;
      } while (!((k > na) || (b_unassignedTracks[k - 1] != x)));

      nb++;
      b_unassignedTracks[nb - 1] = x;
    }

    if (nb < 1) {
      nb = 0;
    }

    b_unassignedTracks.set_size(b_unassignedTracks.size(0), nb);
    std::memset(&toDelete[0], 0, 100U * sizeof(boolean_T));
    i = b_unassignedTracks.size(1);
    for (b_i = 0; b_i < i; b_i++) {
      unsigned int qY;
      boolean_T bv[50];
      boolean_T tentativeTrack;
      u = b_unassignedTracks[b_i];
      temp = pTracksList[static_cast<int>(u) - 1];
      x = pTracksList[static_cast<int>(u) - 1].Age;
      qY = x + 1U;
      if (x + 1U < x) {
        qY = MAX_uint32_T;
      }

      temp.Age = qY;
      pTracksList[static_cast<int>(u) - 1] = temp;
      temp = pTracksList[static_cast<int>(u) - 1];
      temp.IsCoasted = true;
      pTracksList[static_cast<int>(u) - 1] = temp;
      tempLogic = pTrackLogics[static_cast<int>(u) - 1];
      bv[0] = false;
      for (i1 = 0; i1 < 49; i1++) {
        bv[i1 + 1] = tempLogic->pRecentHistory[i1];
      }

      for (i1 = 0; i1 < 50; i1++) {
        tempLogic->pRecentHistory[i1] = bv[i1];
      }

      tempLogic->pIsFirstUpdate = false;
      tempLogic = pTrackLogics[static_cast<int>(u) - 1];
      tentativeTrack = !pTracksList[static_cast<int>(u) - 1].IsConfirmed;
      x = pTracksList[static_cast<int>(u) - 1].Age;
      if (tempLogic->pIsFirstUpdate) {
        toDelete[static_cast<int>(u) - 1] = false;
      } else if (!tentativeTrack) {
        if (x > 5U) {
          boolean_T b_x[5];
          for (i1 = 0; i1 < 5; i1++) {
            b_x[i1] = !tempLogic->pRecentHistory[i1];
          }

          toDelete[static_cast<int>(u) - 1] = ((((b_x[0] + b_x[1]) + b_x[2]) +
            b_x[3]) + b_x[4] >= 5);
        } else {
          if (x < 1U) {
            pEnd = 0;
          } else {
            pEnd = static_cast<int>(x);
          }

          c_x.set_size(1, pEnd);
          for (i1 = 0; i1 < pEnd; i1++) {
            c_x[i1] = !tempLogic->pRecentHistory[i1];
          }

          nb = c_x.size(1);
          if (c_x.size(1) == 0) {
            nz = 0;
          } else {
            nz = c_x[0];
            for (k = 2; k <= nb; k++) {
              nz += c_x[k - 1];
            }
          }

          if (nz < 0) {
            nz = 0;
          }

          toDelete[static_cast<int>(u) - 1] = (static_cast<unsigned int>(nz) >=
            5U);
        }
      } else {
        boolean_T b_x[5];
        for (i1 = 0; i1 < 5; i1++) {
          b_x[i1] = tempLogic->pRecentHistory[i1];
        }

        qY = 5U - x;
        if (5U - x > 5U) {
          qY = 0U;
        }

        toDelete[static_cast<int>(u) - 1] = (5U - static_cast<unsigned int>
          ((((b_x[0] + b_x[1]) + b_x[2]) + b_x[3]) + b_x[4]) > qY);
      }

      if (toDelete[static_cast<int>(u) - 1]) {
        tempLogic = pTrackLogics[static_cast<int>(u) - 1];
        for (i1 = 0; i1 < 50; i1++) {
          tempLogic->pRecentHistory[i1] = false;
        }

        tempLogic->pIsFirstUpdate = true;
      }
    }

    nz = toDelete[0] - 1;
    for (k = 0; k < 99; k++) {
      nz += toDelete[k + 1];
    }

    nb = 0;
    pEnd = 0;
    exitg1 = false;
    while ((!exitg1) && (pEnd < 100)) {
      if (toDelete[pEnd]) {
        nb++;
        ii_data[nb - 1] = static_cast<signed char>(pEnd + 1);
        if (nb >= 100) {
          exitg1 = true;
        } else {
          pEnd++;
        }
      } else {
        pEnd++;
      }
    }

    for (b_i = 0; b_i <= nz; b_i++) {
      unsigned int b_tmp_data[101];
      signed char i2;
      boolean_T tmp_data[101];
      nb = nz - b_i;
      i2 = ii_data[nb];
      temp = pTracksList[i2 - 1];
      tempLogic = pTrackLogics[i2 - 1];
      d = pNumLiveTracks;
      i = static_cast<int>(d + (1.0 - (static_cast<double>(i2) + 1.0)));
      for (j = 0; j < i; j++) {
        x = (static_cast<unsigned int>(i2) + static_cast<unsigned int>(j)) + 1U;
        pTracksList[static_cast<int>(static_cast<double>(x) - 1.0) - 1] =
          pTracksList[static_cast<int>(x) - 1];
        pTrackLogics[static_cast<int>(static_cast<double>(x) - 1.0) - 1] =
          pTrackLogics[static_cast<int>(x) - 1];
      }

      pTracksList[static_cast<int>(pNumLiveTracks) - 1] = temp;
      pTrackLogics[static_cast<int>(pNumLiveTracks) - 1] = tempLogic;
      pNumLiveTracks--;
      i = ii_data[nb] + 1;
      if (i > 100) {
        i1 = 0;
        p = 0;
      } else {
        i1 = i2;
        p = 100;
      }

      pEnd = p - i1;
      nb = pEnd + 1;
      for (p = 0; p < pEnd; p++) {
        tmp_data[p] = pConfirmedTracks[i1 + p];
      }

      tmp_data[pEnd] = false;
      for (i1 = 0; i1 < nb; i1++) {
        pConfirmedTracks[(i2 + i1) - 1] = tmp_data[i1];
      }

      if (i > 100) {
        i = 0;
        i1 = 0;
      } else {
        i = i2;
        i1 = 100;
      }

      pEnd = i1 - i;
      nb = pEnd + 1;
      for (i1 = 0; i1 < pEnd; i1++) {
        b_tmp_data[i1] = pTrackIDs[i + i1];
      }

      b_tmp_data[pEnd] = 0U;
      for (i = 0; i < nb; i++) {
        pTrackIDs[(i2 + i) - 1] = b_tmp_data[i];
      }
    }
  }

  //
  // Arguments    : const array<struct_T, 2U> &localTracks
  //                array<double, 2U> &costMatrix
  // Return Type  : void
  //
  void trackFuser::distance(const array<struct_T, 2U> &localTracks, array<double,
    2U> &costMatrix)
  {
    fuserSourceConfiguration *sourceConfig;
    array<objectTrack, 2U> centralTracks;
    array<double, 3U> S;
    array<double, 3U> allCovars;
    array<double, 3U> centralCovars;
    array<double, 3U> transformedCovars;
    array<double, 2U> allStates;
    array<double, 2U> c;
    array<double, 2U> centralStates;
    array<double, 2U> e;
    array<double, 2U> oneColumnCost;
    array<double, 2U> r;
    array<double, 2U> transformedStates;
    array<int, 2U> r1;
    array<unsigned int, 2U> trackClasses;
    double F[36];
    double numCentralTracks;
    double z_tmp;
    int i;
    int i1;
    int loop_ub_tmp;
    numCentralTracks = pNumLiveTracks;
    i = static_cast<int>(numCentralTracks);
    costMatrix.set_size(i, localTracks.size(1));
    loop_ub_tmp = static_cast<int>(numCentralTracks) * localTracks.size(1);
    for (i1 = 0; i1 < loop_ub_tmp; i1++) {
      costMatrix[i1] = rtInf;
    }

    if (!(pNumLiveTracks == 0.0)) {
      objectTrack obj;
      double d;
      int b_csz_idx_1;
      int b_loop_ub_tmp;
      int csz_idx_1;
      int i2;
      int ibmat;
      int ibtile;
      int jtilecol;
      int loop_ub;
      obj = pTracksList[0];
      centralTracks.set_size(1, i);
      if (static_cast<int>(numCentralTracks) != 0) {
        for (jtilecol = 0; jtilecol < i; jtilecol++) {
          centralTracks[jtilecol] = obj;
        }
      }

      trackClasses.set_size(1, i);
      for (int b_i{0}; b_i < i; b_i++) {
        unsigned int u;
        centralTracks[b_i] = pTracksList[b_i];
        d = std::round(pTracksList[b_i].ObjectClassID);
        if (d < 4.294967296E+9) {
          if (d >= 0.0) {
            u = static_cast<unsigned int>(d);
          } else {
            u = 0U;
          }
        } else if (d >= 4.294967296E+9) {
          u = MAX_uint32_T;
        } else {
          u = 0U;
        }

        trackClasses[b_i] = u;
      }

      centralStates.set_size(6, centralTracks.size(1));
      centralCovars.set_size(6, 6, centralTracks.size(1));
      i1 = centralTracks.size(1);
      for (int b_i{0}; b_i < i1; b_i++) {
        for (i2 = 0; i2 < 6; i2++) {
          centralStates[i2 + 6 * b_i] = centralTracks[b_i].pState[i2];
          for (ibmat = 0; ibmat < 6; ibmat++) {
            centralCovars[(ibmat + 6 * i2) + 36 * b_i] = centralTracks[b_i].
              pStateCovariance[ibmat + 6 * i2];
          }
        }
      }

      obj = pTracksList[0];
      i1 = localTracks.size(1);
      costMatrix.set_size(i, localTracks.size(1));
      b_loop_ub_tmp = 36 * centralCovars.size(2);
      loop_ub = 6 * centralStates.size(1);
      csz_idx_1 = centralStates.size(1);
      b_csz_idx_1 = centralStates.size(1);
      i2 = centralStates.size(1);
      for (int b_i{0}; b_i < i1; b_i++) {
        double Q[9];
        double imvec[6];
        double a_tmp;
        double b_imvec;
        double varargin_1;
        int acoef;
        int iacol;
        boolean_T inKnownIDs[20];
        boolean_T exitg1;
        for (ibmat = 0; ibmat < 9; ibmat++) {
          Q[ibmat] = ProcessNoise[ibmat];
        }

        varargin_1 = localTracks[b_i].UpdateTime - obj.pUpdateTime;
        S.set_size(6, 6, centralCovars.size(2));
        for (ibmat = 0; ibmat < b_loop_ub_tmp; ibmat++) {
          S[ibmat] = centralCovars[ibmat];
        }

        e.set_size(6, centralStates.size(1));
        for (ibmat = 0; ibmat < loop_ub; ibmat++) {
          e[ibmat] = centralStates[ibmat];
        }

        a_tmp = 0.5 * (varargin_1 * varargin_1);
        for (iacol = 0; iacol < 3; iacol++) {
          ibmat = (iacol + 1) << 1;
          oneColumnCost.set_size(1, centralStates.size(1));
          if (csz_idx_1 != 0) {
            ibtile = csz_idx_1 - 1;
            for (int k{0}; k <= ibtile; k++) {
              oneColumnCost[k] = a_tmp * 0.0;
            }
          }

          c.set_size(1, e.size(1));
          if (e.size(1) != 0) {
            acoef = (e.size(1) != 1);
            ibtile = e.size(1) - 1;
            for (int k{0}; k <= ibtile; k++) {
              c[k] = e[(ibmat + 6 * (acoef * k)) - 1] * varargin_1;
            }
          }

          acoef = e.size(1);
          if (e.size(1) == 1) {
            jtilecol = c.size(1);
          } else {
            jtilecol = e.size(1);
          }

          if ((e.size(1) == c.size(1)) && (jtilecol == oneColumnCost.size(1))) {
            c.set_size(1, e.size(1));
            for (ibtile = 0; ibtile < acoef; ibtile++) {
              c[ibtile] = (e[(ibmat + 6 * ibtile) - 2] + c[ibtile]) +
                oneColumnCost[ibtile];
            }

            acoef = c.size(1);
            for (ibtile = 0; ibtile < acoef; ibtile++) {
              e[(ibmat + 6 * ibtile) - 2] = c[ibtile];
            }
          } else {
            binary_expand_op_1(e, iacol + 1, ibmat - 2, c, oneColumnCost);
          }

          c.set_size(1, centralStates.size(1));
          if (b_csz_idx_1 != 0) {
            ibtile = b_csz_idx_1 - 1;
            for (int k{0}; k <= ibtile; k++) {
              c[k] = 0.0 * varargin_1;
            }
          }

          acoef = e.size(1);
          if (e.size(1) == c.size(1)) {
            c.set_size(1, e.size(1));
            for (ibtile = 0; ibtile < acoef; ibtile++) {
              c[ibtile] = e[(ibmat + 6 * ibtile) - 1] + c[ibtile];
            }

            acoef = c.size(1);
            for (ibtile = 0; ibtile < acoef; ibtile++) {
              e[(ibmat + 6 * ibtile) - 1] = c[ibtile];
            }
          } else {
            binary_expand_op(e, iacol + 1, c);
          }
        }

        if (i2 - 1 >= 0) {
          z_tmp = a_tmp * 0.0;
        }

        for (iacol = 0; iacol < i2; iacol++) {
          double b_F[36];
          double c_F[36];
          double U[18];
          double b_U[18];
          double z[6];
          for (ibmat = 0; ibmat < 6; ibmat++) {
            z[ibmat] = centralStates[ibmat + 6 * iacol];
          }

          z[0] = (z[0] + z[1] * varargin_1) + z_tmp;
          z[1] += 0.0 * varargin_1;
          z[2] = (z[2] + z[3] * varargin_1) + z_tmp;
          z[3] += 0.0 * varargin_1;
          z[4] = (z[4] + z[5] * varargin_1) + z_tmp;
          z[5] += 0.0 * varargin_1;
          for (acoef = 0; acoef < 6; acoef++) {
            for (ibmat = 0; ibmat < 6; ibmat++) {
              imvec[ibmat] = centralStates[ibmat + 6 * iacol];
            }

            d = centralStates[acoef + 6 * iacol];
            numCentralTracks = std::fmax(1.4901161193847656E-8,
              1.4901161193847656E-8 * std::abs(d));
            imvec[acoef] = d + numCentralTracks;
            imvec[0] = (imvec[0] + imvec[1] * varargin_1) + z_tmp;
            imvec[1] += 0.0 * varargin_1;
            imvec[2] = (imvec[2] + imvec[3] * varargin_1) + z_tmp;
            imvec[3] += 0.0 * varargin_1;
            imvec[4] = (imvec[4] + imvec[5] * varargin_1) + z_tmp;
            imvec[5] += 0.0 * varargin_1;
            for (ibmat = 0; ibmat < 6; ibmat++) {
              F[ibmat + 6 * acoef] = (imvec[ibmat] - z[ibmat]) /
                numCentralTracks;
            }
          }

          for (ibmat = 0; ibmat < 6; ibmat++) {
            z[ibmat] = centralStates[ibmat + 6 * iacol];
          }

          z[0] = (z[0] + z[1] * varargin_1) + z_tmp;
          z[1] += 0.0 * varargin_1;
          z[2] = (z[2] + z[3] * varargin_1) + z_tmp;
          z[3] += 0.0 * varargin_1;
          z[4] = (z[4] + z[5] * varargin_1) + z_tmp;
          z[5] += 0.0 * varargin_1;
          for (acoef = 0; acoef < 3; acoef++) {
            double specvec_f2[3];
            specvec_f2[0] = 0.0;
            specvec_f2[1] = 0.0;
            specvec_f2[2] = 0.0;
            specvec_f2[acoef] = 1.4901161193847656E-8;
            for (ibmat = 0; ibmat < 6; ibmat++) {
              imvec[ibmat] = centralStates[ibmat + 6 * iacol];
            }

            imvec[0] = (imvec[0] + imvec[1] * varargin_1) + a_tmp * specvec_f2[0];
            imvec[1] += specvec_f2[0] * varargin_1;
            imvec[2] = (imvec[2] + imvec[3] * varargin_1) + a_tmp * specvec_f2[1];
            imvec[3] += specvec_f2[1] * varargin_1;
            imvec[4] = (imvec[4] + imvec[5] * varargin_1) + a_tmp * specvec_f2[2];
            imvec[5] += specvec_f2[2] * varargin_1;
            for (ibmat = 0; ibmat < 6; ibmat++) {
              U[ibmat + 6 * acoef] = (imvec[ibmat] - z[ibmat]) /
                1.4901161193847656E-8;
            }
          }

          for (ibmat = 0; ibmat < 6; ibmat++) {
            for (ibtile = 0; ibtile < 6; ibtile++) {
              d = 0.0;
              for (acoef = 0; acoef < 6; acoef++) {
                d += F[ibmat + 6 * acoef] * S[(acoef + 6 * ibtile) + 36 * iacol];
              }

              b_F[ibmat + 6 * ibtile] = d;
            }

            d = U[ibmat];
            numCentralTracks = U[ibmat + 6];
            b_imvec = U[ibmat + 12];
            for (ibtile = 0; ibtile < 3; ibtile++) {
              b_U[ibmat + 6 * ibtile] = (d * Q[3 * ibtile] + numCentralTracks *
                Q[3 * ibtile + 1]) + b_imvec * Q[3 * ibtile + 2];
            }

            for (ibtile = 0; ibtile < 6; ibtile++) {
              d = 0.0;
              for (acoef = 0; acoef < 6; acoef++) {
                d += b_F[ibmat + 6 * acoef] * F[ibtile + 6 * acoef];
              }

              c_F[ibmat + 6 * ibtile] = d;
            }
          }

          for (ibmat = 0; ibmat < 6; ibmat++) {
            d = b_U[ibmat];
            numCentralTracks = b_U[ibmat + 6];
            b_imvec = b_U[ibmat + 12];
            for (ibtile = 0; ibtile < 6; ibtile++) {
              F[ibmat + 6 * ibtile] = (d * U[ibtile] + numCentralTracks *
                U[ibtile + 6]) + b_imvec * U[ibtile + 12];
            }
          }

          for (ibmat = 0; ibmat < 6; ibmat++) {
            for (ibtile = 0; ibtile < 6; ibtile++) {
              acoef = ibtile + 6 * ibmat;
              S[(ibtile + 6 * ibmat) + 36 * iacol] = c_F[acoef] + F[acoef];
            }
          }
        }

        for (ibmat = 0; ibmat < 20; ibmat++) {
          inKnownIDs[ibmat] = (localTracks[b_i].SourceIndex ==
                               pSourceConfigIDs[ibmat]);
        }

        iacol = 0;
        exitg1 = false;
        while ((!exitg1) && (iacol < 20)) {
          if (inKnownIDs[iacol]) {
            sourceConfig = pSourceConfigurations[iacol];
            exitg1 = true;
          } else {
            iacol++;
          }
        }

        transformedStates.set_size(6, i);
        transformedCovars.set_size(6, 6, i);
        for (jtilecol = 0; jtilecol < i; jtilecol++) {
          acoef = jtilecol * 6;
          ibtile = jtilecol * 36 - 1;
          for (int k{0}; k < 6; k++) {
            transformedStates[acoef + k] = localTracks[b_i].State[k];
            iacol = k * 6;
            ibmat = ibtile + k * 6;
            for (int b_k{0}; b_k < 6; b_k++) {
              transformedCovars[(ibmat + b_k) + 1] = localTracks[b_i].
                StateCovariance[iacol + b_k];
            }
          }
        }

        for (acoef = 0; acoef < i; acoef++) {
          if (!sourceConfig->pIsTransformToLocalValid) {
            sourceConfig->pIsTransformToLocalValid = true;
          }

          for (ibmat = 0; ibmat < 6; ibmat++) {
            transformedStates[ibmat + 6 * acoef] = e[ibmat + 6 * acoef];
            for (ibtile = 0; ibtile < 6; ibtile++) {
              transformedCovars[(ibtile + 6 * ibmat) + 36 * acoef] = S[(ibtile +
                6 * ibmat) + 36 * acoef];
            }
          }
        }

        allStates.set_size(6, transformedStates.size(1) + 1);
        for (ibmat = 0; ibmat < 6; ibmat++) {
          allStates[ibmat] = localTracks[b_i].State[ibmat];
        }

        acoef = transformedStates.size(1);
        for (ibmat = 0; ibmat < acoef; ibmat++) {
          for (ibtile = 0; ibtile < 6; ibtile++) {
            allStates[ibtile + 6 * (ibmat + 1)] = transformedStates[ibtile + 6 *
              ibmat];
          }
        }

        allCovars.set_size(6, 6, transformedCovars.size(2) + 1);
        for (acoef = 0; acoef < 36; acoef++) {
          allCovars[acoef] = localTracks[b_i].StateCovariance[acoef];
        }

        for (acoef = 0; acoef < b_loop_ub_tmp; acoef++) {
          allCovars[acoef + 36] = transformedCovars[acoef];
        }

        oneColumnCost.set_size(1, allStates.size(1));
        acoef = allStates.size(1);
        for (ibmat = 0; ibmat < acoef; ibmat++) {
          oneColumnCost[ibmat] = 0.0;
        }

        e.set_size(6, allStates.size(1));
        ibtile = (allStates.size(1) != 1);
        ibmat = allStates.size(1) - 1;
        for (int k{0}; k <= ibmat; k++) {
          acoef = ibtile * k;
          for (int b_k{0}; b_k < 6; b_k++) {
            e[b_k + 6 * k] = allStates[b_k] - allStates[b_k + 6 * acoef];
          }
        }

        S.set_size(6, 6, allCovars.size(2));
        for (int k{0}; k <= ibmat; k++) {
          acoef = ibtile * k;
          for (int b_k{0}; b_k < 6; b_k++) {
            for (iacol = 0; iacol < 6; iacol++) {
              S[(iacol + 6 * b_k) + 36 * k] = allCovars[iacol + 6 * b_k] +
                allCovars[(iacol + 6 * b_k) + 36 * acoef];
            }
          }
        }

        r.set_size(1, allStates.size(1));
        ibmat = allStates.size(1);
        for (iacol = 0; iacol < ibmat; iacol++) {
          int ipiv[6];
          boolean_T isodd;
          for (ibtile = 0; ibtile < 6; ibtile++) {
            for (acoef = 0; acoef < 6; acoef++) {
              F[acoef + 6 * ibtile] = S[(acoef + 6 * ibtile) + 36 * iacol];
            }
          }

          internal::reflapack::xzgetrf(F, ipiv);
          numCentralTracks = F[0];
          isodd = false;
          for (int k{0}; k < 5; k++) {
            numCentralTracks *= F[(k + 6 * (k + 1)) + 1];
            if (ipiv[k] > k + 1) {
              isodd = !isodd;
            }
          }

          if (isodd) {
            numCentralTracks = -numCentralTracks;
          }

          for (ibtile = 0; ibtile < 6; ibtile++) {
            imvec[ibtile] = e[ibtile + 6 * iacol];
          }

          std::copy(&(*(double (*)[36])&S[36 * iacol])[0], &(*(double (*)[36])&
                     S[36 * iacol])[36], &F[0]);
          internal::mrdiv(imvec, F);
          b_imvec = 0.0;
          for (ibtile = 0; ibtile < 6; ibtile++) {
            b_imvec += imvec[ibtile] * e[ibtile + 6 * iacol];
          }

          r[iacol] = b_imvec + std::log(numCentralTracks);
        }

        acoef = r.size(1);
        for (ibmat = 0; ibmat < acoef; ibmat++) {
          oneColumnCost[ibmat] = r[ibmat];
        }

        ibmat = (oneColumnCost.size(1) >= 2);
        acoef = costMatrix.size(0);
        for (ibtile = 0; ibtile < acoef; ibtile++) {
          costMatrix[ibtile + costMatrix.size(0) * b_i] = oneColumnCost[ibmat +
            ibtile];
        }

        if (localTracks[b_i].ObjectClassID != 0.0) {
          acoef = trackClasses.size(1);
          oneColumnCost.set_size(1, trackClasses.size(1));
          for (ibmat = 0; ibmat < acoef; ibmat++) {
            oneColumnCost[ibmat] = trackClasses[ibmat];
          }

          ibtile = oneColumnCost.size(1) - 1;
          acoef = 0;
          for (iacol = 0; iacol <= ibtile; iacol++) {
            d = oneColumnCost[iacol];
            if ((d != localTracks[b_i].ObjectClassID) && (d != 0.0)) {
              acoef++;
            }
          }

          r1.set_size(1, acoef);
          acoef = 0;
          for (iacol = 0; iacol <= ibtile; iacol++) {
            d = oneColumnCost[iacol];
            if ((d != localTracks[b_i].ObjectClassID) && (d != 0.0)) {
              r1[acoef] = iacol;
              acoef++;
            }
          }

          acoef = r1.size(1);
          for (ibmat = 0; ibmat < acoef; ibmat++) {
            costMatrix[r1[ibmat] + costMatrix.size(0) * b_i] = rtInf;
          }
        }
      }

      ibtile = loop_ub_tmp - 1;
      for (int b_i{0}; b_i <= ibtile; b_i++) {
        if (costMatrix[b_i] > 5.0) {
          costMatrix[b_i] = rtInf;
        }
      }
    }
  }

  //
  // Arguments    : unsigned int in_TrackID
  //                unsigned int in_SourceIndex
  //                double in_UpdateTime
  //                const double in_State[6]
  //                const double in_StateCovariance[36]
  //                double in_ObjectClassID
  //                const char in_TrackLogic[7]
  //                objectTrack *out
  // Return Type  : void
  //
  void trackFuser::ensureTrack(unsigned int in_TrackID, unsigned int
    in_SourceIndex, double in_UpdateTime, const double in_State[6], const double
    in_StateCovariance[36], double in_ObjectClassID, const char in_TrackLogic[7],
    objectTrack *out)
  {
    static const char cv[128]{ '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
      '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f', '\x10',
      '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19',
      '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#', '$',
      '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3',
      '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a', 'b',
      'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
      'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_', '`',
      'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
      'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~',
      '\x7f' };

    static const char cv1[7]{ 'H', 'i', 's', 't', 'o', 'r', 'y' };

    int i;
    boolean_T b_bool;
    out->pTrackLogicState.set_size(1, 0);
    out->TrackID = in_TrackID;
    out->SourceIndex = in_SourceIndex;
    out->BranchID = 0U;
    out->pUpdateTime = in_UpdateTime;
    out->Age = 1U;
    for (i = 0; i < 6; i++) {
      out->pState[i] = in_State[i];
    }

    std::copy(&in_StateCovariance[0], &in_StateCovariance[36],
              &out->pStateCovariance[0]);
    out->IsConfirmed = true;
    out->IsCoasted = false;
    out->IsSelfReported = true;
    out->ObjectClassProbabilities = 1.0;
    out->ObjectClassID = in_ObjectClassID;
    b_bool = false;
    i = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (i < 7) {
        if (cv[static_cast<int>(in_TrackLogic[i])] != cv[static_cast<int>(cv1[i])])
        {
          exitg1 = 1;
        } else {
          i++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      out->pTrackLogicState.set_size(1, 1);
      out->pTrackLogicState[0] = true;
    }
  }

  //
  // Arguments    : const array<struct_T, 2U> &localTracks
  //                const array<unsigned int, 2U> &assignments
  // Return Type  : void
  //
  void trackFuser::fuseAssigned(const array<struct_T, 2U> &localTracks, const
    array<unsigned int, 2U> &assignments)
  {
    fuserSourceConfiguration *thisConfig;
    objectTrack obj;
    trackHistoryLogic *b_obj;
    array<objectTrack, 2U> transformedTracks;
    array<double, 1U> b_assignedSourceInds;
    array<int, 1U> assignedSourceInds;
    array<unsigned int, 1U> b_assignments;
    array<int, 1U> r;
    array<int, 1U> r1;
    array<unsigned int, 1U> uniqueAssigned;
    array<boolean_T, 1U> inAssigned;
    array<boolean_T, 1U> toFuse;
    int c_i;
    int i;
    int k;
    int ntilecols;
    int nz;
    boolean_T exitg1;
    obj = pTracksList[0];
    transformedTracks.set_size(1, assignments.size(0));
    if (assignments.size(0) != 0) {
      ntilecols = assignments.size(0);
      for (nz = 0; nz < ntilecols; nz++) {
        transformedTracks[nz] = obj;
      }
    }

    i = assignments.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      struct_T expl_temp;
      boolean_T inKnownIDs[20];
      expl_temp = localTracks[static_cast<int>(assignments[b_i +
        assignments.size(0)]) - 1];
      for (k = 0; k < 20; k++) {
        inKnownIDs[k] = (expl_temp.SourceIndex == pSourceConfigIDs[k]);
      }

      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 20)) {
        if (inKnownIDs[c_i]) {
          thisConfig = pSourceConfigurations[c_i];
          exitg1 = true;
        } else {
          c_i++;
        }
      }

      if (!thisConfig->pIsTransformToCentralValid) {
        thisConfig->pIsTransformToCentralValid = true;
      }

      trackFuser::ensureTrack(expl_temp.TrackID, expl_temp.SourceIndex,
        expl_temp.UpdateTime, expl_temp.State, expl_temp.StateCovariance,
        expl_temp.ObjectClassID, expl_temp.TrackLogic, &transformedTracks[b_i]);
    }

    b_assignments.set_size(assignments.size(0));
    ntilecols = assignments.size(0);
    for (i = 0; i < ntilecols; i++) {
      b_assignments[i] = assignments[i];
    }

    unique_vector(b_assignments, uniqueAssigned);
    i = uniqueAssigned.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      boolean_T y;
      ntilecols = assignments.size(0);
      inAssigned.set_size(assignments.size(0));
      for (k = 0; k < ntilecols; k++) {
        inAssigned[k] = (uniqueAssigned[b_i] == assignments[k]);
      }

      eml_find(inAssigned, assignedSourceInds);
      toFuse.set_size(assignedSourceInds.size(0));
      k = assignedSourceInds.size(0);
      for (c_i = 0; c_i < k; c_i++) {
        if ((transformedTracks[assignedSourceInds[c_i] - 1].IsConfirmed &&
             (!transformedTracks[assignedSourceInds[c_i] - 1].IsCoasted)) ||
            transformedTracks[assignedSourceInds[c_i] - 1].IsSelfReported) {
          toFuse[c_i] = true;
        } else {
          toFuse[c_i] = false;
        }
      }

      y = false;
      ntilecols = 1;
      exitg1 = false;
      while ((!exitg1) && (ntilecols <= toFuse.size(0))) {
        if (toFuse[ntilecols - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ntilecols++;
        }
      }

      if (y) {
        double id;
        int obj_tmp_tmp;
        unsigned int u;
        ntilecols = toFuse.size(0);
        if (toFuse.size(0) == 0) {
          nz = 0;
        } else {
          nz = toFuse[0];
          for (k = 2; k <= ntilecols; k++) {
            nz += toFuse[k - 1];
          }
        }

        obj_tmp_tmp = static_cast<int>(uniqueAssigned[b_i]) - 1;
        obj = pTracksList[obj_tmp_tmp];
        id = static_cast<double>(pTracksList[obj_tmp_tmp].Age) + static_cast<
          double>(nz);
        if (id < 4.294967296E+9) {
          if (id >= 0.0) {
            u = static_cast<unsigned int>(id);
          } else {
            u = 0U;
          }
        } else {
          u = MAX_uint32_T;
        }

        obj.Age = u;
        pTracksList[obj_tmp_tmp] = obj;
        nz = inAssigned.size(0) - 1;
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (inAssigned[c_i]) {
            ntilecols++;
          }
        }

        r.set_size(ntilecols);
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (inAssigned[c_i]) {
            r[ntilecols] = c_i;
            ntilecols++;
          }
        }

        id = pTracksList[obj_tmp_tmp].ObjectClassID;
        ntilecols = 0;
        while ((id == 0.0) && (ntilecols < r.size(0))) {
          ntilecols++;
          id = localTracks[static_cast<int>(assignments[r[ntilecols - 1] +
            assignments.size(0)]) - 1].ObjectClassID;
        }

        obj = pTracksList[obj_tmp_tmp];
        obj.ObjectClassID = id;
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        nz = toFuse.size(0) - 1;
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (toFuse[c_i]) {
            ntilecols++;
          }
        }

        r1.set_size(ntilecols);
        ntilecols = 0;
        for (c_i = 0; c_i <= nz; c_i++) {
          if (toFuse[c_i]) {
            r1[ntilecols] = c_i;
            ntilecols++;
          }
        }

        ntilecols = r1.size(0);
        b_assignedSourceInds.set_size(r1.size(0));
        for (k = 0; k < ntilecols; k++) {
          b_assignedSourceInds[k] = assignedSourceInds[r1[k]];
        }

        cFuser.fuse(obj, transformedTracks, b_assignedSourceInds);
        pTracksList[obj_tmp_tmp] = obj;
        ntilecols = toFuse.size(0);
        if (toFuse.size(0) == 0) {
          nz = 0;
        } else {
          nz = toFuse[0];
          for (k = 2; k <= ntilecols; k++) {
            nz += toFuse[k - 1];
          }
        }

        for (ntilecols = 0; ntilecols < nz; ntilecols++) {
          boolean_T bv[50];
          b_obj = pTrackLogics[obj_tmp_tmp];
          bv[0] = true;
          for (k = 0; k < 49; k++) {
            bv[k + 1] = b_obj->pRecentHistory[k];
          }

          for (k = 0; k < 50; k++) {
            b_obj->pRecentHistory[k] = bv[k];
          }

          b_obj->pIsFirstUpdate = false;
        }

        if (pTracksList[obj_tmp_tmp].IsConfirmed) {
          y = true;
        } else {
          b_obj = pTrackLogics[obj_tmp_tmp];
          if (b_obj->pIsFirstUpdate) {
            y = false;
          } else {
            boolean_T x[5];
            for (k = 0; k < 5; k++) {
              x[k] = b_obj->pRecentHistory[k];
            }

            y = ((((x[0] + x[1]) + x[2]) + x[3]) + x[4] >= 5);
          }

          if (y || (pTracksList[obj_tmp_tmp].ObjectClassID > 0.0)) {
            y = true;
          } else {
            y = false;
          }
        }

        obj = pTracksList[obj_tmp_tmp];
        obj.IsConfirmed = y;
        pTracksList[obj_tmp_tmp] = obj;
        pConfirmedTracks[obj_tmp_tmp] = pTracksList[obj_tmp_tmp].IsConfirmed;
        obj = pTracksList[obj_tmp_tmp];
        obj.IsCoasted = false;
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        ntilecols = r.size(0);
        b_assignments.set_size(r.size(0));
        for (k = 0; k < ntilecols; k++) {
          b_assignments[k] = assignments[r[k] + assignments.size(0)];
        }

        obj.IsSelfReported = getSelfReporting(localTracks, b_assignments);
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        pTracksList[obj_tmp_tmp] = obj;
        obj = pTracksList[obj_tmp_tmp];
        pTracksList[obj_tmp_tmp] = obj;
      }
    }
  }

  //
  // Arguments    : objectTrack &centralTrack
  //                const array<objectTrack, 2U> &sourceTracks
  //                const array<double, 1U> &inAssigned
  // Return Type  : void
  //
  namespace fusion
  {
    namespace internal
    {
      void Fuserxcov::fuse(objectTrack &centralTrack, const array<objectTrack,
                           2U> &sourceTracks, const array<double, 1U>
                           &inAssigned) const
      {
        array<double, 3U> allCovars;
        array<double, 2U> A;
        array<double, 2U> CovMatrix;
        array<double, 2U> allStates;
        array<double, 2U> allTimes;
        array<double, 2U> b_A;
        array<double, 2U> uniqueTimes;
        array<double, 1U> B;
        array<double, 1U> b_B;
        array<int, 2U> b_I;
        array<int, 2U> ii;
        array<unsigned int, 2U> tracksAtThisTime;
        array<boolean_T, 2U> x;
        double b_centralTrack[36];
        double trackTime;
        int exponent;
        int i;
        int ibtile;
        int nx;
        allTimes.set_size(1, inAssigned.size(0));
        nx = inAssigned.size(0);
        for (i = 0; i < nx; i++) {
          allTimes[i] = sourceTracks[static_cast<int>(inAssigned[0]) - 1].
            pUpdateTime;
        }

        i = inAssigned.size(0);
        for (ibtile = 0; ibtile <= i - 2; ibtile++) {
          allTimes[ibtile + 1] = sourceTracks[static_cast<int>(inAssigned[ibtile
            + 1]) - 1].pUpdateTime;
        }

        ::coder::internal::sort(allTimes, ii);
        b_I.set_size(1, ii.size(1));
        nx = ii.size(1);
        for (i = 0; i < nx; i++) {
          b_I[i] = ii[i];
        }

        unique_vector(allTimes, uniqueTimes);
        trackTime = centralTrack.pUpdateTime;
        i = uniqueTimes.size(1);
        for (int j{0}; j < i; j++) {
          double initialFusedState[6];
          double s;
          int b_k;
          int i1;
          int i2;
          int ibmat;
          int idx;
          int jtilecol;
          int k;
          boolean_T exitg1;
          s = uniqueTimes[j];
          gaussEKFilter::predict(centralTrack.pState,
            centralTrack.pStateCovariance, ProcessNoise, s - trackTime,
            initialFusedState);
          nx = allTimes.size(1);
          x.set_size(1, allTimes.size(1));
          for (i1 = 0; i1 < nx; i1++) {
            x[i1] = (s == allTimes[i1]);
          }

          nx = x.size(1);
          idx = 0;
          ii.set_size(1, x.size(1));
          ibtile = 0;
          exitg1 = false;
          while ((!exitg1) && (ibtile <= nx - 1)) {
            if (x[ibtile]) {
              idx++;
              ii[idx - 1] = ibtile + 1;
              if (idx >= nx) {
                exitg1 = true;
              } else {
                ibtile++;
              }
            } else {
              ibtile++;
            }
          }

          if (x.size(1) == 1) {
            if (idx == 0) {
              ii.set_size(1, 0);
            }
          } else {
            if (idx < 1) {
              idx = 0;
            }

            ii.set_size(ii.size(0), idx);
          }

          tracksAtThisTime.set_size(1, ii.size(1));
          nx = ii.size(1);
          for (i1 = 0; i1 < nx; i1++) {
            tracksAtThisTime[i1] = static_cast<unsigned int>(ii[i1]);
          }

          i1 = static_cast<int>(static_cast<unsigned int>(tracksAtThisTime.size
            (1)) + 1U);
          allStates.set_size(6, static_cast<int>(static_cast<unsigned int>
            (tracksAtThisTime.size(1)) + 1U));
          allCovars.set_size(6, 6, static_cast<int>(static_cast<unsigned int>
            (tracksAtThisTime.size(1)) + 1U));
          for (jtilecol = 0; jtilecol < i1; jtilecol++) {
            nx = jtilecol * 6;
            ibtile = jtilecol * 36 - 1;
            for (k = 0; k < 6; k++) {
              allStates[nx + k] = initialFusedState[k];
              idx = k * 6;
              ibmat = ibtile + k * 6;
              for (b_k = 0; b_k < 6; b_k++) {
                allCovars[(ibmat + b_k) + 1] = centralTrack.pStateCovariance[idx
                  + b_k];
              }
            }
          }

          i1 = tracksAtThisTime.size(1);
          for (k = 0; k < i1; k++) {
            nx = static_cast<int>(inAssigned[b_I[static_cast<int>
                                  (tracksAtThisTime[k]) - 1] - 1]) - 1;
            for (i2 = 0; i2 < 6; i2++) {
              allStates[i2 + 6 * (k + 1)] = sourceTracks[nx].pState[i2];
              for (idx = 0; idx < 6; idx++) {
                allCovars[(idx + 6 * i2) + 36 * (k + 1)] = sourceTracks[nx].
                  pStateCovariance[idx + 6 * i2];
              }
            }
          }

          i1 = allCovars.size(2);
          for (ibtile = 0; ibtile < i1; ibtile++) {
            for (k = 0; k < 6; k++) {
              centralTrack.pState[k] = std::abs(allCovars[(k + 6 * k) + 36 *
                ibtile]);
            }

            if (!std::isnan(centralTrack.pState[0])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 7)) {
                if (!std::isnan(centralTrack.pState[k - 1])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              trackTime = centralTrack.pState[0];
            } else {
              trackTime = centralTrack.pState[idx - 1];
              i2 = idx + 1;
              for (k = i2; k < 7; k++) {
                s = centralTrack.pState[k - 1];
                if (trackTime < s) {
                  trackTime = s;
                }
              }
            }

            if ((!std::isinf(trackTime)) && (!std::isnan(trackTime)) &&
                (!(trackTime < 4.4501477170144028E-308))) {
              std::frexp(trackTime, &exponent);
            }
          }

          if (allCovars.size(2) == 1) {
            for (i1 = 0; i1 < 6; i1++) {
              centralTrack.pState[i1] = allStates[i1];
              for (i2 = 0; i2 < 6; i2++) {
                centralTrack.pStateCovariance[i2 + 6 * i1] = allCovars[i2 + 6 *
                  i1];
              }
            }
          } else {
            CovMatrix.set_size(1, allCovars.size(2));
            i1 = allCovars.size(2);
            for (ibtile = 0; ibtile < i1; ibtile++) {
              int ipiv[6];
              boolean_T isodd;
              for (i2 = 0; i2 < 6; i2++) {
                for (idx = 0; idx < 6; idx++) {
                  centralTrack.pStateCovariance[idx + 6 * i2] = allCovars[(idx +
                    6 * i2) + 36 * ibtile];
                }
              }

              ::coder::internal::reflapack::xzgetrf
                (centralTrack.pStateCovariance, ipiv);
              trackTime = centralTrack.pStateCovariance[0];
              isodd = false;
              for (k = 0; k < 5; k++) {
                trackTime *= centralTrack.pStateCovariance[(k + 6 * (k + 1)) + 1];
                if (ipiv[k] > k + 1) {
                  isodd = !isodd;
                }
              }

              if (isodd) {
                trackTime = -trackTime;
              }

              CovMatrix[ibtile] = trackTime;
            }

            A.set_size(allCovars.size(2) - 1, allCovars.size(2));
            nx = (allCovars.size(2) - 1) * allCovars.size(2);
            for (i1 = 0; i1 < nx; i1++) {
              A[i1] = 0.0;
            }

            i1 = allCovars.size(2);
            for (ibtile = 0; ibtile <= i1 - 2; ibtile++) {
              A[ibtile + A.size(0) * ibtile] = CovMatrix[ibtile];
              A[ibtile + A.size(0) * (ibtile + 1)] = -CovMatrix[ibtile + 1];
            }

            b_A.set_size(A.size(0) + 1, A.size(1));
            nx = A.size(1);
            for (i1 = 0; i1 < nx; i1++) {
              ibtile = A.size(0);
              for (i2 = 0; i2 < ibtile; i2++) {
                b_A[i2 + b_A.size(0) * i1] = A[i2 + A.size(0) * i1];
              }
            }

            nx = A.size(1);
            for (i1 = 0; i1 < nx; i1++) {
              b_A[A.size(0) + b_A.size(0) * i1] = 1.0;
            }

            B.set_size(allCovars.size(2));
            nx = allCovars.size(2);
            for (i1 = 0; i1 <= nx - 2; i1++) {
              B[i1] = 0.0;
            }

            B[allCovars.size(2) - 1] = 1.0;
            if (b_A.size(0) == b_A.size(1)) {
              int LDA;
              int n;
              jtilecol = b_A.size(0);
              n = b_A.size(1);
              if (jtilecol <= n) {
                n = jtilecol;
              }

              jtilecol = B.size(0);
              if (jtilecol <= n) {
                n = jtilecol;
              }

              LDA = b_A.size(0);
              ii.set_size(1, n);
              ii[0] = 1;
              nx = 1;
              for (k = 2; k <= n; k++) {
                nx++;
                ii[k - 1] = nx;
              }

              jtilecol = n - 1;
              if (jtilecol > n) {
                jtilecol = n;
              }

              for (int b_j{0}; b_j < jtilecol; b_j++) {
                int b_tmp;
                b_k = n - b_j;
                b_tmp = b_j * (LDA + 1);
                ibmat = b_tmp + 2;
                if (b_k < 1) {
                  nx = -1;
                } else {
                  nx = 0;
                  if (b_k > 1) {
                    trackTime = std::abs(b_A[b_tmp]);
                    for (k = 2; k <= b_k; k++) {
                      s = std::abs(b_A[(b_tmp + k) - 1]);
                      if (s > trackTime) {
                        nx = k - 1;
                        trackTime = s;
                      }
                    }
                  }
                }

                if (b_A[b_tmp + nx] != 0.0) {
                  if (nx != 0) {
                    idx = b_j + nx;
                    ii[b_j] = idx + 1;
                    for (k = 0; k < n; k++) {
                      nx = k * LDA;
                      ibtile = b_j + nx;
                      trackTime = b_A[ibtile];
                      i1 = idx + nx;
                      b_A[ibtile] = b_A[i1];
                      b_A[i1] = trackTime;
                    }
                  }

                  i1 = b_tmp + b_k;
                  for (ibtile = ibmat; ibtile <= i1; ibtile++) {
                    b_A[ibtile - 1] = b_A[ibtile - 1] / b_A[b_tmp];
                  }
                }

                nx = b_tmp + LDA;
                idx = nx;
                for (ibtile = 0; ibtile <= b_k - 2; ibtile++) {
                  trackTime = b_A[nx + ibtile * LDA];
                  if (trackTime != 0.0) {
                    i1 = idx + 2;
                    i2 = b_k + idx;
                    for (ibmat = i1; ibmat <= i2; ibmat++) {
                      b_A[ibmat - 1] = b_A[ibmat - 1] + b_A[((b_tmp + ibmat) -
                        idx) - 1] * -trackTime;
                    }
                  }

                  idx += LDA;
                }
              }

              LDA = b_A.size(0);
              for (ibtile = 0; ibtile <= n - 2; ibtile++) {
                i1 = ii[ibtile];
                if (i1 != ibtile + 1) {
                  nx = static_cast<int>(B[ibtile]);
                  B[ibtile] = B[i1 - 1];
                  B[i1 - 1] = nx;
                }
              }

              for (k = 0; k < n; k++) {
                nx = LDA * k;
                if (B[k] != 0.0) {
                  i1 = k + 2;
                  for (ibtile = i1; ibtile <= n; ibtile++) {
                    B[ibtile - 1] = B[ibtile - 1] - B[k] * b_A[(ibtile + nx) - 1];
                  }
                }
              }

              for (k = n; k >= 1; k--) {
                nx = LDA * (k - 1);
                s = B[k - 1];
                if (s != 0.0) {
                  B[k - 1] = s / b_A[(k + nx) - 1];
                  for (ibtile = 0; ibtile <= k - 2; ibtile++) {
                    B[ibtile] = B[ibtile] - B[k - 1] * b_A[ibtile + nx];
                  }
                }
              }
            } else {
              b_B.set_size(B.size(0));
              nx = B.size(0) - 1;
              for (i1 = 0; i1 <= nx; i1++) {
                b_B[i1] = B[i1];
              }

              ::coder::internal::qrsolve(b_A, b_B, B);
            }

            std::memset(&centralTrack.pStateCovariance[0], 0, 36U * sizeof
                        (double));
            i1 = allCovars.size(2);
            for (ibtile = 0; ibtile < i1; ibtile++) {
              mldivide(&allCovars[36 * ibtile], b_centralTrack);
              for (i2 = 0; i2 < 36; i2++) {
                centralTrack.pStateCovariance[i2] += B[ibtile] *
                  b_centralTrack[i2];
              }
            }

            std::copy(&centralTrack.pStateCovariance[0],
                      &centralTrack.pStateCovariance[36], &b_centralTrack[0]);
            mldivide(b_centralTrack, centralTrack.pStateCovariance);
            for (ibtile = 0; ibtile < 6; ibtile++) {
              initialFusedState[ibtile] = 0.0;
            }

            i1 = allCovars.size(2);
            for (ibtile = 0; ibtile < i1; ibtile++) {
              mldivide(&allCovars[36 * ibtile], b_centralTrack);
              for (i2 = 0; i2 < 6; i2++) {
                s = 0.0;
                for (idx = 0; idx < 6; idx++) {
                  s += B[ibtile] * b_centralTrack[i2 + 6 * idx] * allStates[idx
                    + 6 * ibtile];
                }

                initialFusedState[i2] += s;
              }
            }

            for (i1 = 0; i1 < 6; i1++) {
              centralTrack.pState[i1] = 0.0;
              for (i2 = 0; i2 < 6; i2++) {
                centralTrack.pState[i1] += centralTrack.pStateCovariance[i1 + 6 *
                  i2] * initialFusedState[i2];
              }
            }
          }

          trackTime = uniqueTimes[j];
        }

        ensurePosDefMatrix(centralTrack.pStateCovariance);
        centralTrack.pUpdateTime = trackTime;
      }

      //
      // Arguments    : const double x[6]
      //                double P[36]
      //                const double Q[9]
      //                double varargin_1
      //                double xk[6]
      // Return Type  : void
      //
      void gaussEKFilter::predict(const double x[6], double P[36], const double
        Q[9], double varargin_1, double xk[6])
      {
        double F[36];
        double b_F[36];
        double U[18];
        double b_U[18];
        double imvec[6];
        double z[6];
        double d;
        double xk_tmp;
        double xk_tmp_tmp;
        xk_tmp_tmp = 0.5 * (varargin_1 * varargin_1);
        xk_tmp = xk_tmp_tmp * 0.0;
        for (int i{0}; i < 6; i++) {
          xk[i] = x[i];
          z[i] = x[i];
        }

        xk[0] = (xk[0] + xk[1] * varargin_1) + xk_tmp;
        xk[1] += 0.0 * varargin_1;
        xk[2] = (xk[2] + xk[3] * varargin_1) + xk_tmp;
        xk[3] += 0.0 * varargin_1;
        xk[4] = (xk[4] + xk[5] * varargin_1) + xk_tmp;
        xk[5] += 0.0 * varargin_1;
        z[0] = (z[0] + z[1] * varargin_1) + xk_tmp;
        z[1] += 0.0 * varargin_1;
        z[2] = (z[2] + z[3] * varargin_1) + xk_tmp;
        z[3] += 0.0 * varargin_1;
        z[4] = (z[4] + z[5] * varargin_1) + xk_tmp;
        z[5] += 0.0 * varargin_1;
        for (int j{0}; j < 6; j++) {
          double epsilon;
          for (int i{0}; i < 6; i++) {
            imvec[i] = x[i];
          }

          d = x[j];
          epsilon = std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::
                              abs(d));
          imvec[j] = d + epsilon;
          imvec[0] = (imvec[0] + imvec[1] * varargin_1) + xk_tmp;
          imvec[1] += 0.0 * varargin_1;
          imvec[2] = (imvec[2] + imvec[3] * varargin_1) + xk_tmp;
          imvec[3] += 0.0 * varargin_1;
          imvec[4] = (imvec[4] + imvec[5] * varargin_1) + xk_tmp;
          imvec[5] += 0.0 * varargin_1;
          for (int b_i{0}; b_i < 6; b_i++) {
            F[b_i + 6 * j] = (imvec[b_i] - z[b_i]) / epsilon;
          }
        }

        for (int i{0}; i < 6; i++) {
          z[i] = x[i];
        }

        z[0] = (z[0] + z[1] * varargin_1) + xk_tmp;
        z[1] += 0.0 * varargin_1;
        z[2] = (z[2] + z[3] * varargin_1) + xk_tmp;
        z[3] += 0.0 * varargin_1;
        z[4] = (z[4] + z[5] * varargin_1) + xk_tmp;
        z[5] += 0.0 * varargin_1;
        for (int j{0}; j < 3; j++) {
          double specvec_f2[3];
          specvec_f2[0] = 0.0;
          specvec_f2[1] = 0.0;
          specvec_f2[2] = 0.0;
          specvec_f2[j] = 1.4901161193847656E-8;
          for (int i{0}; i < 6; i++) {
            imvec[i] = x[i];
          }

          imvec[0] = (imvec[0] + imvec[1] * varargin_1) + xk_tmp_tmp *
            specvec_f2[0];
          imvec[1] += specvec_f2[0] * varargin_1;
          imvec[2] = (imvec[2] + imvec[3] * varargin_1) + xk_tmp_tmp *
            specvec_f2[1];
          imvec[3] += specvec_f2[1] * varargin_1;
          imvec[4] = (imvec[4] + imvec[5] * varargin_1) + xk_tmp_tmp *
            specvec_f2[2];
          imvec[5] += specvec_f2[2] * varargin_1;
          for (int b_i{0}; b_i < 6; b_i++) {
            U[b_i + 6 * j] = (imvec[b_i] - z[b_i]) / 1.4901161193847656E-8;
          }
        }

        for (int b_i{0}; b_i < 6; b_i++) {
          for (int i{0}; i < 6; i++) {
            d = 0.0;
            for (int j{0}; j < 6; j++) {
              d += F[b_i + 6 * j] * P[j + 6 * i];
            }

            b_F[b_i + 6 * i] = d;
          }

          d = U[b_i];
          xk_tmp_tmp = U[b_i + 6];
          xk_tmp = U[b_i + 12];
          for (int i{0}; i < 3; i++) {
            b_U[b_i + 6 * i] = (d * Q[3 * i] + xk_tmp_tmp * Q[3 * i + 1]) +
              xk_tmp * Q[3 * i + 2];
          }
        }

        for (int b_i{0}; b_i < 6; b_i++) {
          for (int i{0}; i < 6; i++) {
            d = 0.0;
            for (int j{0}; j < 6; j++) {
              d += b_F[b_i + 6 * j] * F[i + 6 * j];
            }

            P[b_i + 6 * i] = d;
          }
        }

        for (int b_i{0}; b_i < 6; b_i++) {
          d = b_U[b_i];
          xk_tmp_tmp = b_U[b_i + 6];
          xk_tmp = b_U[b_i + 12];
          for (int i{0}; i < 6; i++) {
            F[b_i + 6 * i] = (d * U[i] + xk_tmp_tmp * U[i + 6]) + xk_tmp * U[i +
              12];
          }
        }

        for (int b_i{0}; b_i < 36; b_i++) {
          P[b_i] += F[b_i];
        }
      }

      //
      // Arguments    : const array<struct_T, 2U> &localTracks
      //                const array<unsigned int, 1U> &assignedlocalTracks
      // Return Type  : boolean_T
      //
    }
  }

  boolean_T trackFuser::getSelfReporting(const array<struct_T, 2U> &localTracks,
    const array<unsigned int, 1U> &assignedlocalTracks)
  {
    fuserSourceConfiguration *thisSource;
    int i;
    boolean_T tf;
    tf = false;
    i = 0;
    while ((!tf) && (i < assignedlocalTracks.size(0))) {
      int b_i;
      boolean_T inKnownIDs[20];
      boolean_T exitg1;
      i++;
      for (b_i = 0; b_i < 20; b_i++) {
        inKnownIDs[b_i] = (localTracks[static_cast<int>(assignedlocalTracks[i -
          1]) - 1].SourceIndex == pSourceConfigIDs[b_i]);
      }

      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i < 20)) {
        if (inKnownIDs[b_i]) {
          thisSource = pSourceConfigurations[b_i];
          exitg1 = true;
        } else {
          b_i++;
        }
      }

      tf = thisSource->IsInternalSource;
    }

    return tf;
  }

  //
  // Arguments    : const array<struct_T, 2U> &localTracks
  //                array<unsigned int, 1U> &unassignedlocalTracks
  // Return Type  : void
  //
  void trackFuser::initializeCentralTracks(const array<struct_T, 2U>
    &localTracks, array<unsigned int, 1U> &unassignedlocalTracks)
  {
    fuserSourceConfiguration *b_thisSource;
    fuserSourceConfiguration *thisConfig;
    fuserSourceConfiguration *thisSource;
    objectTrack centralTrack;
    trackHistoryLogic *obj;
    array<double, 2U> costMatrix;
    array<double, 2U> r1;
    array<unsigned int, 2U> a;
    array<unsigned int, 1U> allSourceInds;
    array<unsigned int, 1U> checkedUnassigned;
    array<int, 1U> r;
    array<boolean_T, 2U> assignedToNewTrack;
    double S[72];
    double P[36];
    int i;
    int j;
    unsigned int u;
    boolean_T exitg1;
    allSourceInds.set_size(unassignedlocalTracks.size(0));
    j = unassignedlocalTracks.size(0);
    for (i = 0; i < j; i++) {
      allSourceInds[i] = localTracks[static_cast<int>(unassignedlocalTracks[i])
        - 1].SourceIndex;
    }

    exitg1 = false;
    while ((!exitg1) && ((unassignedlocalTracks.size(0) > 0) && (pNumLiveTracks <
             100.0))) {
      double x[6];
      double c_i;
      int b_i;
      unsigned int centralTrack_tmp;
      int end_tmp;
      int nz;
      int partialTrueCount;
      unsigned int q0;
      unsigned int qY;
      int trueCount;
      char cv[7];
      boolean_T bv[50];
      boolean_T inKnownIDs[20];
      boolean_T exitg2;
      boolean_T isodd;
      boolean_T tf;
      for (b_i = 0; b_i < 20; b_i++) {
        inKnownIDs[b_i] = (localTracks[static_cast<int>(unassignedlocalTracks[0])
                           - 1].SourceIndex == pSourceConfigIDs[b_i]);
      }

      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 20)) {
        if (inKnownIDs[i]) {
          thisConfig = pSourceConfigurations[i];
          exitg2 = true;
        } else {
          i++;
        }
      }

      if (!thisConfig->pIsTransformToCentralValid) {
        thisConfig->pIsTransformToCentralValid = true;
      }

      nz = static_cast<int>(unassignedlocalTracks[0]) - 1;
      centralTrack_tmp = localTracks[static_cast<int>(unassignedlocalTracks[0])
        - 1].SourceIndex;
      for (int i1{0}; i1 < 6; i1++) {
        x[i1] = localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].
          State[i1];
      }

      std::copy(&localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].
                StateCovariance[0], &localTracks[static_cast<int>
                (unassignedlocalTracks[0]) - 1].StateCovariance[36], &P[0]);
      for (int i2{0}; i2 < 7; i2++) {
        cv[i2] = localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].
          TrackLogic[i2];
      }

      trackFuser::ensureTrack(localTracks[static_cast<int>
        (unassignedlocalTracks[0]) - 1].TrackID, centralTrack_tmp, localTracks[
        static_cast<int>(unassignedlocalTracks[0]) - 1].UpdateTime, x, P,
        localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].
        ObjectClassID, cv, &centralTrack);
      pNumLiveTracks++;
      q0 = pLastTrackID;
      qY = q0 + 1U;
      if (q0 + 1U < q0) {
        qY = MAX_uint32_T;
      }

      pLastTrackID = qY;
      centralTrack.TrackID = pLastTrackID;
      centralTrack.SourceIndex = 1U;
      centralTrack.Age = 1U;
      pTracksList[static_cast<int>(pNumLiveTracks) - 1] = centralTrack;
      tf = false;
      i = 0;
      while ((!tf) && (i < 1)) {
        i = 1;
        for (b_i = 0; b_i < 20; b_i++) {
          inKnownIDs[b_i] = (centralTrack_tmp == pSourceConfigIDs[b_i]);
        }

        partialTrueCount = 0;
        exitg2 = false;
        while ((!exitg2) && (partialTrueCount < 20)) {
          if (inKnownIDs[partialTrueCount]) {
            thisSource = pSourceConfigurations[partialTrueCount];
            exitg2 = true;
          } else {
            partialTrueCount++;
          }
        }

        tf = thisSource->IsInternalSource;
      }

      centralTrack.IsSelfReported = tf;
      c_i = pNumLiveTracks;
      c_i = pTracksList[static_cast<int>(c_i) - 1].ObjectClassID;
      j = 0;
      while ((c_i == 0.0) && (j < 1)) {
        j = 1;
        c_i = localTracks[nz].ObjectClassID;
      }

      centralTrack.ObjectClassID = c_i;
      obj = pTrackLogics[static_cast<int>(pNumLiveTracks) - 1];
      bv[0] = true;
      for (b_i = 0; b_i < 49; b_i++) {
        bv[b_i + 1] = obj->pRecentHistory[b_i];
      }

      for (b_i = 0; b_i < 50; b_i++) {
        obj->pRecentHistory[b_i] = bv[b_i];
      }

      obj->pIsFirstUpdate = false;
      obj = pTrackLogics[static_cast<int>(pNumLiveTracks) - 1];
      if (obj->pIsFirstUpdate) {
        isodd = false;
      } else {
        boolean_T b_x[5];
        for (b_i = 0; b_i < 5; b_i++) {
          b_x[b_i] = obj->pRecentHistory[b_i];
        }

        isodd = ((((b_x[0] + b_x[1]) + b_x[2]) + b_x[3]) + b_x[4] >= 5);
      }

      if (isodd || (c_i > 0.0)) {
        isodd = true;
      } else {
        isodd = false;
      }

      centralTrack.IsConfirmed = isodd;
      pTrackIDs[static_cast<int>(pNumLiveTracks) - 1] = pLastTrackID;
      pTracksList[static_cast<int>(pNumLiveTracks) - 1] = centralTrack;
      pConfirmedTracks[static_cast<int>(pNumLiveTracks) - 1] = isodd;
      costMatrix.set_size(1, unassignedlocalTracks.size(0));
      j = unassignedlocalTracks.size(0);
      for (b_i = 0; b_i < j; b_i++) {
        costMatrix[b_i] = 0.0;
      }

      end_tmp = allSourceInds.size(0) - 1;
      for (i = 0; i <= end_tmp; i++) {
        if (allSourceInds[i] == localTracks[nz].SourceIndex) {
          costMatrix[i] = rtInf;
        }
      }

      trueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (allSourceInds[i] != localTracks[nz].SourceIndex) {
          trueCount++;
        }
      }

      r.set_size(trueCount);
      partialTrueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (allSourceInds[i] != localTracks[nz].SourceIndex) {
          r[partialTrueCount] = i;
          partialTrueCount++;
        }
      }

      if (r.size(0) == 0) {
        r1.set_size(1, 0);
      } else {
        r1.set_size(1, r.size(0));
        j = r.size(0);
        for (b_i = 0; b_i < j; b_i++) {
          r1[b_i] = 0.0;
        }

        b_i = r.size(0);
        for (i = 0; i < b_i; i++) {
          struct_T expl_temp;
          double allCovars[72];
          double allStates[12];
          double e[12];
          double allCosts[2];
          j = static_cast<int>(unassignedlocalTracks[r[i]]) - 1;
          expl_temp = localTracks[j];
          std::copy(&centralTrack.pStateCovariance[0],
                    &centralTrack.pStateCovariance[36], &P[0]);
          fusion::internal::gaussEKFilter::predict(centralTrack.pState, P,
            ProcessNoise, localTracks[static_cast<int>(unassignedlocalTracks[r[i]])
            - 1].UpdateTime - centralTrack.pUpdateTime, x);
          for (trueCount = 0; trueCount < 20; trueCount++) {
            inKnownIDs[trueCount] = (localTracks[j].SourceIndex ==
              pSourceConfigIDs[trueCount]);
          }

          partialTrueCount = 0;
          exitg2 = false;
          while ((!exitg2) && (partialTrueCount < 20)) {
            if (inKnownIDs[partialTrueCount]) {
              b_thisSource = pSourceConfigurations[partialTrueCount];
              exitg2 = true;
            } else {
              partialTrueCount++;
            }
          }

          if (!b_thisSource->pIsTransformToLocalValid) {
            b_thisSource->pIsTransformToLocalValid = true;
          }

          for (j = 0; j < 6; j++) {
            allStates[j] = x[j];
            allStates[j + 6] = expl_temp.State[j];
          }

          for (j = 0; j < 36; j++) {
            allCovars[j] = P[j];
            allCovars[j + 36] = expl_temp.StateCovariance[j];
          }

          for (end_tmp = 0; end_tmp < 2; end_tmp++) {
            for (nz = 0; nz < 6; nz++) {
              j = nz + 6 * end_tmp;
              e[j] = allStates[nz] - allStates[j];
              for (partialTrueCount = 0; partialTrueCount < 6; partialTrueCount
                   ++) {
                j = partialTrueCount + 6 * nz;
                trueCount = j + 36 * end_tmp;
                S[trueCount] = allCovars[j] + allCovars[trueCount];
              }
            }
          }

          for (partialTrueCount = 0; partialTrueCount < 2; partialTrueCount++) {
            double c_x;
            int ipiv[6];
            std::copy(&S[partialTrueCount * 36], &S[static_cast<int>(
                       static_cast<unsigned int>(partialTrueCount * 36) + 36U)],
                      &P[0]);
            internal::reflapack::xzgetrf(P, ipiv);
            c_i = P[0];
            isodd = false;
            for (end_tmp = 0; end_tmp < 5; end_tmp++) {
              c_i *= P[(end_tmp + 6 * (end_tmp + 1)) + 1];
              if (ipiv[end_tmp] > end_tmp + 1) {
                isodd = !isodd;
              }
            }

            if (isodd) {
              c_i = -c_i;
            }

            for (trueCount = 0; trueCount < 6; trueCount++) {
              x[trueCount] = e[trueCount + 6 * partialTrueCount];
            }

            internal::mrdiv(x, &S[36 * partialTrueCount]);
            c_x = 0.0;
            for (trueCount = 0; trueCount < 6; trueCount++) {
              c_x += x[trueCount] * e[trueCount + 6 * partialTrueCount];
            }

            allCosts[partialTrueCount] = c_x + std::log(c_i);
          }

          r1[i] = allCosts[1];
        }
      }

      j = r.size(0) - 1;
      for (b_i = 0; b_i <= j; b_i++) {
        costMatrix[r[b_i]] = r1[b_i];
      }

      checkedUnassigned.set_size(unassignedlocalTracks.size(0));
      j = unassignedlocalTracks.size(0);
      assignedToNewTrack.set_size(1, costMatrix.size(1));
      for (b_i = 0; b_i < j; b_i++) {
        checkedUnassigned[b_i] = unassignedlocalTracks[b_i];
        assignedToNewTrack[b_i] = (costMatrix[b_i] < 5.0);
      }

      isodd = false;
      j = 1;
      exitg2 = false;
      while ((!exitg2) && (j <= assignedToNewTrack.size(1))) {
        if (assignedToNewTrack[j - 1]) {
          isodd = true;
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (isodd) {
        j = assignedToNewTrack.size(1);
        nz = assignedToNewTrack[0];
        for (end_tmp = 2; end_tmp <= j; end_tmp++) {
          nz += assignedToNewTrack[end_tmp - 1];
        }

        c_i = pNumLiveTracks;
        end_tmp = assignedToNewTrack.size(1) - 1;
        trueCount = 0;
        for (i = 0; i <= end_tmp; i++) {
          if (assignedToNewTrack[i]) {
            trueCount++;
          }
        }

        checkedUnassigned.set_size(trueCount);
        partialTrueCount = 0;
        for (i = 0; i <= end_tmp; i++) {
          if (assignedToNewTrack[i]) {
            checkedUnassigned[partialTrueCount] = unassignedlocalTracks[i];
            partialTrueCount++;
          }
        }

        a.set_size(nz, 2);
        if (nz - 1 >= 0) {
          c_i = std::round(c_i);
          if (c_i < 4.294967296E+9) {
            if (c_i >= 0.0) {
              u = static_cast<unsigned int>(c_i);
            } else {
              u = 0U;
            }
          } else if (c_i >= 4.294967296E+9) {
            u = MAX_uint32_T;
          } else {
            u = 0U;
          }
        }

        for (b_i = 0; b_i < nz; b_i++) {
          a[b_i] = u;
        }

        j = checkedUnassigned.size(0);
        for (b_i = 0; b_i < j; b_i++) {
          a[b_i + a.size(0)] = checkedUnassigned[b_i];
        }

        fuseAssigned(localTracks, a);
        trueCount = 0;
        for (i = 0; i <= end_tmp; i++) {
          if (!assignedToNewTrack[i]) {
            trueCount++;
          }
        }

        checkedUnassigned.set_size(trueCount);
        partialTrueCount = 0;
        for (i = 0; i <= end_tmp; i++) {
          if (!assignedToNewTrack[i]) {
            checkedUnassigned[partialTrueCount] = unassignedlocalTracks[i];
            partialTrueCount++;
          }
        }

        trueCount = 0;
        partialTrueCount = 0;
        for (i = 0; i <= end_tmp; i++) {
          if (!assignedToNewTrack[i]) {
            j = trueCount + 1;
            trueCount++;
            allSourceInds[partialTrueCount] = allSourceInds[i];
            partialTrueCount = j;
          }
        }

        allSourceInds.set_size(trueCount);
        if (tf || pTracksList[static_cast<int>(pNumLiveTracks) - 1].
            IsSelfReported) {
          isodd = true;
        } else {
          isodd = false;
        }

        b_i = static_cast<int>(pNumLiveTracks) - 1;
        centralTrack = pTracksList[b_i];
        centralTrack.IsSelfReported = isodd;
        pTracksList[b_i] = centralTrack;
      }

      j = checkedUnassigned.size(0);
      for (end_tmp = 0; end_tmp <= j - 2; end_tmp++) {
        checkedUnassigned[end_tmp] = checkedUnassigned[end_tmp + 1];
      }

      if (checkedUnassigned.size(0) - 1 < 1) {
        b_i = 0;
      } else {
        b_i = checkedUnassigned.size(0) - 1;
      }

      checkedUnassigned.set_size(b_i);
      j = allSourceInds.size(0);
      for (end_tmp = 0; end_tmp <= j - 2; end_tmp++) {
        allSourceInds[end_tmp] = allSourceInds[end_tmp + 1];
      }

      if (allSourceInds.size(0) - 1 < 1) {
        b_i = 0;
      } else {
        b_i = allSourceInds.size(0) - 1;
      }

      allSourceInds.set_size(b_i);
      if (allSourceInds.size(0) != 0) {
        unassignedlocalTracks.set_size(checkedUnassigned.size(0));
        j = checkedUnassigned.size(0);
        for (b_i = 0; b_i < j; b_i++) {
          unassignedlocalTracks[b_i] = checkedUnassigned[b_i];
        }
      } else {
        exitg1 = true;
      }
    }
  }

  //
  // Arguments    : int i
  //                const array<double, 1U> &dist
  // Return Type  : void
  //
  namespace matlab
  {
    namespace internal
    {
      namespace coder
      {
        void minPriorityQueue::percUp(int i, const array<double, 1U> &dist)
        {
          int iparent;
          boolean_T exitg1;
          iparent = i / 2 - 1;
          exitg1 = false;
          while ((!exitg1) && (iparent + 1 > 0)) {
            double d;
            double d1;
            int obj_idx_1;
            obj_idx_1 = heap[i - 1];
            d = dist[obj_idx_1 - 1];
            d1 = dist[heap[iparent] - 1];
            if ((d < d1) || ((d == d1) && (obj_idx_1 <= heap[iparent]))) {
              obj_idx_1 = heap[i - 1];
              heap[i - 1] = heap[iparent];
              heap[iparent] = obj_idx_1;
              obj_idx_1 = indexToHeap[heap[i - 1] - 1];
              indexToHeap[heap[i - 1] - 1] = indexToHeap[heap[iparent] - 1];
              indexToHeap[heap[iparent] - 1] = obj_idx_1;
              i = iparent + 1;
              iparent = (iparent + 1) / 2 - 1;
            } else {
              exitg1 = true;
            }
          }
        }

        //
        // Arguments    : void
        // Return Type  : void
        //
      }
    }
  }

  void trackFuser::matlabCodegenDestructor()
  {
    if (!matlabCodegenIsDeleted) {
      matlabCodegenIsDeleted = true;
      if (isInitialized == 1) {
        isInitialized = 2;
        releaseWrapper();
      }
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void trackFuser::releaseWrapper()
  {
    trackHistoryLogic *obj;
    if (isSetupComplete) {
      pNumLiveTracks = 0.0;
      for (int i{0}; i < 100; i++) {
        pTrackIDs[i] = 0U;
      }

      for (int i{0}; i < 100; i++) {
        pConfirmedTracks[i] = false;
      }

      if (cAssigner.isInitialized == 1) {
        cAssigner.isInitialized = 2;
      }

      obj = pTrackLogics[0];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[1];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[2];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[3];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[4];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[5];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[6];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[7];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[8];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[9];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[10];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[11];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[12];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[13];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[14];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[15];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[16];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[17];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[18];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[19];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[20];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[21];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[22];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[23];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[24];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[25];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[26];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[27];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[28];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[29];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[30];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[31];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[32];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[33];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[34];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[35];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[36];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[37];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[38];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[39];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[40];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[41];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[42];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[43];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[44];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[45];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[46];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[47];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[48];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[49];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[50];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[51];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[52];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[53];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[54];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[55];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[56];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[57];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[58];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[59];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[60];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[61];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[62];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[63];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[64];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[65];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[66];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[67];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[68];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[69];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[70];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[71];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[72];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[73];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[74];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[75];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[76];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[77];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[78];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[79];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[80];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[81];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[82];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[83];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[84];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[85];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[86];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[87];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[88];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[89];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[90];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[91];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[92];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[93];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[94];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[95];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[96];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[97];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[98];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      obj = pTrackLogics[99];
      for (int i{0}; i < 50; i++) {
        obj->pRecentHistory[i] = false;
      }

      obj->pIsFirstUpdate = true;
      pLastTrackID = 0U;
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void trackFuser::resetImpl()
  {
    trackHistoryLogic *obj;
    pNumLiveTracks = 0.0;
    pLastTimeStamp = -2.2204460492503131E-16;
    for (int i{0}; i < 100; i++) {
      pTrackIDs[i] = 0U;
    }

    for (int i{0}; i < 100; i++) {
      pConfirmedTracks[i] = false;
    }

    for (int i{0}; i < 20; i++) {
      pIsValidSource[i] = false;
    }

    obj = pTrackLogics[0];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[1];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[2];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[3];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[4];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[5];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[6];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[7];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[8];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[9];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[10];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[11];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[12];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[13];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[14];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[15];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[16];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[17];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[18];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[19];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[20];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[21];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[22];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[23];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[24];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[25];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[26];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[27];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[28];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[29];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[30];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[31];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[32];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[33];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[34];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[35];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[36];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[37];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[38];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[39];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[40];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[41];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[42];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[43];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[44];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[45];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[46];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[47];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[48];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[49];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[50];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[51];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[52];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[53];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[54];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[55];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[56];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[57];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[58];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[59];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[60];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[61];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[62];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[63];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[64];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[65];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[66];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[67];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[68];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[69];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[70];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[71];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[72];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[73];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[74];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[75];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[76];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[77];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[78];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[79];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[80];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[81];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[82];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[83];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[84];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[85];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[86];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[87];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[88];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[89];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[90];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[91];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[92];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[93];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[94];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[95];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[96];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[97];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[98];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[99];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }

    obj->pIsFirstUpdate = true;
    pLastTrackID = 0U;
  }

  //
  // Arguments    : const array<struct_T, 2U> &tracks
  // Return Type  : void
  //
  void trackFuser::setupImpl(const array<struct_T, 2U> &tracks)
  {
    fuserSourceConfiguration *source;
    fuserSourceConfiguration *sourceConfig;
    objectTrack b[100];
    objectTrack centralTrack;
    int i;
    boolean_T inKnownIDs[20];
    boolean_T exitg1;
    for (i = 0; i < 20; i++) {
      inKnownIDs[i] = (tracks[0].SourceIndex == pSourceConfigIDs[i]);
    }

    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 20)) {
      if (inKnownIDs[i]) {
        sourceConfig = pSourceConfigurations[i];
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (!sourceConfig->pIsTransformToCentralValid) {
      sourceConfig->pIsTransformToCentralValid = true;
    }

    for (i = 0; i < 20; i++) {
      inKnownIDs[i] = (tracks[0].SourceIndex == pSourceConfigIDs[i]);
    }

    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 20)) {
      if (inKnownIDs[i]) {
        source = pSourceConfigurations[i];
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (!source->pIsTransformToCentralValid) {
      source->pIsTransformToCentralValid = true;
    }

    centralTrack.TrackID = MAX_uint32_T;
    centralTrack.BranchID = MAX_uint32_T;
    centralTrack.SourceIndex = 1U;
    centralTrack.pUpdateTime = 0.0;
    centralTrack.Age = MAX_uint32_T;
    for (i = 0; i < 6; i++) {
      centralTrack.pState[i] = 0.0;
    }

    for (i = 0; i < 36; i++) {
      centralTrack.pStateCovariance[i] = iv[i];
    }

    centralTrack.IsConfirmed = false;
    centralTrack.IsCoasted = false;
    centralTrack.IsSelfReported = false;
    centralTrack.ObjectClassProbabilities = 1.0;
    centralTrack.ObjectClassID = 0.0;
    centralTrack.pTrackLogicState.set_size(1, 5);
    for (i = 0; i < 5; i++) {
      centralTrack.pTrackLogicState[i] = false;
    }

    for (i = 0; i < 100; i++) {
      b[i] = centralTrack;
    }

    for (i = 0; i < 100; i++) {
      pTracksList[i] = b[i];
    }

    pUsedConfigIDs.set_size(1, 20);
    for (i = 0; i < 20; i++) {
      pUsedConfigIDs[i] = false;
    }

    pUsedConfigIDs.set_size(2, 20);
    for (i = 0; i < 40; i++) {
      pUsedConfigIDs[i] = false;
    }

    pUsedConfigIDs.set_size(1, 20);
    for (i = 0; i < 20; i++) {
      pUsedConfigIDs[i] = false;
    }
  }

  //
  // Arguments    : const array<struct_T, 2U> &varargin_1
  //                double varargin_2
  //                struct3_T varargout_1_data[]
  //                struct3_T varargout_2_data[]
  //                array<struct3_T, 1U> &varargout_3
  //                unsigned int varargout_4_TrackIDsAtStepBeginning_data[]
  //                int varargout_4_TrackIDsAtStepBeginning_size[2]
  //                array<double, 2U> &varargout_4_CostMatrix
  //                array<unsigned int, 2U> &varargout_4_Assignments
  //                array<unsigned int, 1U> &varargout_4_UnassignedCentralTracks
  //                array<unsigned int, 1U> &varargout_4_UnassignedLocalTracks
  //                array<unsigned int, 1U> &varargout_4_NonInitializingLocalTracks
  //                unsigned int varargout_4_InitializedCentralTrackIDs_data[]
  //                int varargout_4_InitializedCentralTrackIDs_size[2]
  //                array<unsigned int, 2U> &varargout_4_UpdatedCentralTrackIDs
  //                unsigned int varargout_4_DeletedTrackIDs_data[]
  //                int varargout_4_DeletedTrackIDs_size[2]
  //                unsigned int varargout_4_TrackIDsAtStepEnd_data[]
  //                int varargout_4_TrackIDsAtStepEnd_size[2]
  //                int &varargout_2_size
  // Return Type  : int
  //
  int trackFuser::step(const array<struct_T, 2U> &varargin_1, double varargin_2,
                       struct3_T varargout_1_data[], struct3_T varargout_2_data[],
                       array<struct3_T, 1U> &varargout_3, unsigned int
                       varargout_4_TrackIDsAtStepBeginning_data[], int
                       varargout_4_TrackIDsAtStepBeginning_size[2], array<double,
                       2U> &varargout_4_CostMatrix, array<unsigned int, 2U>
                       &varargout_4_Assignments, array<unsigned int, 1U>
                       &varargout_4_UnassignedCentralTracks, array<unsigned int,
                       1U> &varargout_4_UnassignedLocalTracks, array<unsigned
                       int, 1U> &varargout_4_NonInitializingLocalTracks,
                       unsigned int varargout_4_InitializedCentralTrackIDs_data[],
                       int varargout_4_InitializedCentralTrackIDs_size[2], array<
                       unsigned int, 2U> &varargout_4_UpdatedCentralTrackIDs,
                       unsigned int varargout_4_DeletedTrackIDs_data[], int
                       varargout_4_DeletedTrackIDs_size[2], unsigned int
                       varargout_4_TrackIDsAtStepEnd_data[], int
                       varargout_4_TrackIDsAtStepEnd_size[2], int
                       &varargout_2_size)
  {
    unsigned int inSize[8];
    int k;
    boolean_T exitg1;
    if (isInitialized != 1) {
      cell_wrap_3 varSizes[2];
      double Q[9];
      isSetupComplete = false;
      isInitialized = 1;
      varSizes[0].f1[0] = 1U;
      varSizes[0].f1[1] = static_cast<unsigned int>(varargin_1.size(1));
      for (k = 0; k < 6; k++) {
        varSizes[0].f1[k + 2] = 1U;
      }

      for (k = 0; k < 8; k++) {
        varSizes[1].f1[k] = 1U;
      }

      inputVarSize[0] = varSizes[0];
      inputVarSize[1] = varSizes[1];
      setupImpl(varargin_1);
      cAssigner.isInitialized = 0;
      for (k = 0; k < 9; k++) {
        Q[k] = ProcessNoise[k];
      }

      for (k = 0; k < 9; k++) {
        cFuser.ProcessNoise[k] = Q[k];
      }

      for (k = 0; k < 50; k++) {
        _pobj0[0].pRecentHistory[k] = false;
      }

      pTrackLogics[0] = &_pobj0[0];
      for (k = 0; k < 50; k++) {
        _pobj0[1].pRecentHistory[k] = false;
      }

      pTrackLogics[1] = &_pobj0[1];
      for (k = 0; k < 50; k++) {
        _pobj0[2].pRecentHistory[k] = false;
      }

      pTrackLogics[2] = &_pobj0[2];
      for (k = 0; k < 50; k++) {
        _pobj0[3].pRecentHistory[k] = false;
      }

      pTrackLogics[3] = &_pobj0[3];
      for (k = 0; k < 50; k++) {
        _pobj0[4].pRecentHistory[k] = false;
      }

      pTrackLogics[4] = &_pobj0[4];
      for (k = 0; k < 50; k++) {
        _pobj0[5].pRecentHistory[k] = false;
      }

      pTrackLogics[5] = &_pobj0[5];
      for (k = 0; k < 50; k++) {
        _pobj0[6].pRecentHistory[k] = false;
      }

      pTrackLogics[6] = &_pobj0[6];
      for (k = 0; k < 50; k++) {
        _pobj0[7].pRecentHistory[k] = false;
      }

      pTrackLogics[7] = &_pobj0[7];
      for (k = 0; k < 50; k++) {
        _pobj0[8].pRecentHistory[k] = false;
      }

      pTrackLogics[8] = &_pobj0[8];
      for (k = 0; k < 50; k++) {
        _pobj0[9].pRecentHistory[k] = false;
      }

      pTrackLogics[9] = &_pobj0[9];
      for (k = 0; k < 50; k++) {
        _pobj0[10].pRecentHistory[k] = false;
      }

      pTrackLogics[10] = &_pobj0[10];
      for (k = 0; k < 50; k++) {
        _pobj0[11].pRecentHistory[k] = false;
      }

      pTrackLogics[11] = &_pobj0[11];
      for (k = 0; k < 50; k++) {
        _pobj0[12].pRecentHistory[k] = false;
      }

      pTrackLogics[12] = &_pobj0[12];
      for (k = 0; k < 50; k++) {
        _pobj0[13].pRecentHistory[k] = false;
      }

      pTrackLogics[13] = &_pobj0[13];
      for (k = 0; k < 50; k++) {
        _pobj0[14].pRecentHistory[k] = false;
      }

      pTrackLogics[14] = &_pobj0[14];
      for (k = 0; k < 50; k++) {
        _pobj0[15].pRecentHistory[k] = false;
      }

      pTrackLogics[15] = &_pobj0[15];
      for (k = 0; k < 50; k++) {
        _pobj0[16].pRecentHistory[k] = false;
      }

      pTrackLogics[16] = &_pobj0[16];
      for (k = 0; k < 50; k++) {
        _pobj0[17].pRecentHistory[k] = false;
      }

      pTrackLogics[17] = &_pobj0[17];
      for (k = 0; k < 50; k++) {
        _pobj0[18].pRecentHistory[k] = false;
      }

      pTrackLogics[18] = &_pobj0[18];
      for (k = 0; k < 50; k++) {
        _pobj0[19].pRecentHistory[k] = false;
      }

      pTrackLogics[19] = &_pobj0[19];
      for (k = 0; k < 50; k++) {
        _pobj0[20].pRecentHistory[k] = false;
      }

      pTrackLogics[20] = &_pobj0[20];
      for (k = 0; k < 50; k++) {
        _pobj0[21].pRecentHistory[k] = false;
      }

      pTrackLogics[21] = &_pobj0[21];
      for (k = 0; k < 50; k++) {
        _pobj0[22].pRecentHistory[k] = false;
      }

      pTrackLogics[22] = &_pobj0[22];
      for (k = 0; k < 50; k++) {
        _pobj0[23].pRecentHistory[k] = false;
      }

      pTrackLogics[23] = &_pobj0[23];
      for (k = 0; k < 50; k++) {
        _pobj0[24].pRecentHistory[k] = false;
      }

      pTrackLogics[24] = &_pobj0[24];
      for (k = 0; k < 50; k++) {
        _pobj0[25].pRecentHistory[k] = false;
      }

      pTrackLogics[25] = &_pobj0[25];
      for (k = 0; k < 50; k++) {
        _pobj0[26].pRecentHistory[k] = false;
      }

      pTrackLogics[26] = &_pobj0[26];
      for (k = 0; k < 50; k++) {
        _pobj0[27].pRecentHistory[k] = false;
      }

      pTrackLogics[27] = &_pobj0[27];
      for (k = 0; k < 50; k++) {
        _pobj0[28].pRecentHistory[k] = false;
      }

      pTrackLogics[28] = &_pobj0[28];
      for (k = 0; k < 50; k++) {
        _pobj0[29].pRecentHistory[k] = false;
      }

      pTrackLogics[29] = &_pobj0[29];
      for (k = 0; k < 50; k++) {
        _pobj0[30].pRecentHistory[k] = false;
      }

      pTrackLogics[30] = &_pobj0[30];
      for (k = 0; k < 50; k++) {
        _pobj0[31].pRecentHistory[k] = false;
      }

      pTrackLogics[31] = &_pobj0[31];
      for (k = 0; k < 50; k++) {
        _pobj0[32].pRecentHistory[k] = false;
      }

      pTrackLogics[32] = &_pobj0[32];
      for (k = 0; k < 50; k++) {
        _pobj0[33].pRecentHistory[k] = false;
      }

      pTrackLogics[33] = &_pobj0[33];
      for (k = 0; k < 50; k++) {
        _pobj0[34].pRecentHistory[k] = false;
      }

      pTrackLogics[34] = &_pobj0[34];
      for (k = 0; k < 50; k++) {
        _pobj0[35].pRecentHistory[k] = false;
      }

      pTrackLogics[35] = &_pobj0[35];
      for (k = 0; k < 50; k++) {
        _pobj0[36].pRecentHistory[k] = false;
      }

      pTrackLogics[36] = &_pobj0[36];
      for (k = 0; k < 50; k++) {
        _pobj0[37].pRecentHistory[k] = false;
      }

      pTrackLogics[37] = &_pobj0[37];
      for (k = 0; k < 50; k++) {
        _pobj0[38].pRecentHistory[k] = false;
      }

      pTrackLogics[38] = &_pobj0[38];
      for (k = 0; k < 50; k++) {
        _pobj0[39].pRecentHistory[k] = false;
      }

      pTrackLogics[39] = &_pobj0[39];
      for (k = 0; k < 50; k++) {
        _pobj0[40].pRecentHistory[k] = false;
      }

      pTrackLogics[40] = &_pobj0[40];
      for (k = 0; k < 50; k++) {
        _pobj0[41].pRecentHistory[k] = false;
      }

      pTrackLogics[41] = &_pobj0[41];
      for (k = 0; k < 50; k++) {
        _pobj0[42].pRecentHistory[k] = false;
      }

      pTrackLogics[42] = &_pobj0[42];
      for (k = 0; k < 50; k++) {
        _pobj0[43].pRecentHistory[k] = false;
      }

      pTrackLogics[43] = &_pobj0[43];
      for (k = 0; k < 50; k++) {
        _pobj0[44].pRecentHistory[k] = false;
      }

      pTrackLogics[44] = &_pobj0[44];
      for (k = 0; k < 50; k++) {
        _pobj0[45].pRecentHistory[k] = false;
      }

      pTrackLogics[45] = &_pobj0[45];
      for (k = 0; k < 50; k++) {
        _pobj0[46].pRecentHistory[k] = false;
      }

      pTrackLogics[46] = &_pobj0[46];
      for (k = 0; k < 50; k++) {
        _pobj0[47].pRecentHistory[k] = false;
      }

      pTrackLogics[47] = &_pobj0[47];
      for (k = 0; k < 50; k++) {
        _pobj0[48].pRecentHistory[k] = false;
      }

      pTrackLogics[48] = &_pobj0[48];
      for (k = 0; k < 50; k++) {
        _pobj0[49].pRecentHistory[k] = false;
      }

      pTrackLogics[49] = &_pobj0[49];
      for (k = 0; k < 50; k++) {
        _pobj0[50].pRecentHistory[k] = false;
      }

      pTrackLogics[50] = &_pobj0[50];
      for (k = 0; k < 50; k++) {
        _pobj0[51].pRecentHistory[k] = false;
      }

      pTrackLogics[51] = &_pobj0[51];
      for (k = 0; k < 50; k++) {
        _pobj0[52].pRecentHistory[k] = false;
      }

      pTrackLogics[52] = &_pobj0[52];
      for (k = 0; k < 50; k++) {
        _pobj0[53].pRecentHistory[k] = false;
      }

      pTrackLogics[53] = &_pobj0[53];
      for (k = 0; k < 50; k++) {
        _pobj0[54].pRecentHistory[k] = false;
      }

      pTrackLogics[54] = &_pobj0[54];
      for (k = 0; k < 50; k++) {
        _pobj0[55].pRecentHistory[k] = false;
      }

      pTrackLogics[55] = &_pobj0[55];
      for (k = 0; k < 50; k++) {
        _pobj0[56].pRecentHistory[k] = false;
      }

      pTrackLogics[56] = &_pobj0[56];
      for (k = 0; k < 50; k++) {
        _pobj0[57].pRecentHistory[k] = false;
      }

      pTrackLogics[57] = &_pobj0[57];
      for (k = 0; k < 50; k++) {
        _pobj0[58].pRecentHistory[k] = false;
      }

      pTrackLogics[58] = &_pobj0[58];
      for (k = 0; k < 50; k++) {
        _pobj0[59].pRecentHistory[k] = false;
      }

      pTrackLogics[59] = &_pobj0[59];
      for (k = 0; k < 50; k++) {
        _pobj0[60].pRecentHistory[k] = false;
      }

      pTrackLogics[60] = &_pobj0[60];
      for (k = 0; k < 50; k++) {
        _pobj0[61].pRecentHistory[k] = false;
      }

      pTrackLogics[61] = &_pobj0[61];
      for (k = 0; k < 50; k++) {
        _pobj0[62].pRecentHistory[k] = false;
      }

      pTrackLogics[62] = &_pobj0[62];
      for (k = 0; k < 50; k++) {
        _pobj0[63].pRecentHistory[k] = false;
      }

      pTrackLogics[63] = &_pobj0[63];
      for (k = 0; k < 50; k++) {
        _pobj0[64].pRecentHistory[k] = false;
      }

      pTrackLogics[64] = &_pobj0[64];
      for (k = 0; k < 50; k++) {
        _pobj0[65].pRecentHistory[k] = false;
      }

      pTrackLogics[65] = &_pobj0[65];
      for (k = 0; k < 50; k++) {
        _pobj0[66].pRecentHistory[k] = false;
      }

      pTrackLogics[66] = &_pobj0[66];
      for (k = 0; k < 50; k++) {
        _pobj0[67].pRecentHistory[k] = false;
      }

      pTrackLogics[67] = &_pobj0[67];
      for (k = 0; k < 50; k++) {
        _pobj0[68].pRecentHistory[k] = false;
      }

      pTrackLogics[68] = &_pobj0[68];
      for (k = 0; k < 50; k++) {
        _pobj0[69].pRecentHistory[k] = false;
      }

      pTrackLogics[69] = &_pobj0[69];
      for (k = 0; k < 50; k++) {
        _pobj0[70].pRecentHistory[k] = false;
      }

      pTrackLogics[70] = &_pobj0[70];
      for (k = 0; k < 50; k++) {
        _pobj0[71].pRecentHistory[k] = false;
      }

      pTrackLogics[71] = &_pobj0[71];
      for (k = 0; k < 50; k++) {
        _pobj0[72].pRecentHistory[k] = false;
      }

      pTrackLogics[72] = &_pobj0[72];
      for (k = 0; k < 50; k++) {
        _pobj0[73].pRecentHistory[k] = false;
      }

      pTrackLogics[73] = &_pobj0[73];
      for (k = 0; k < 50; k++) {
        _pobj0[74].pRecentHistory[k] = false;
      }

      pTrackLogics[74] = &_pobj0[74];
      for (k = 0; k < 50; k++) {
        _pobj0[75].pRecentHistory[k] = false;
      }

      pTrackLogics[75] = &_pobj0[75];
      for (k = 0; k < 50; k++) {
        _pobj0[76].pRecentHistory[k] = false;
      }

      pTrackLogics[76] = &_pobj0[76];
      for (k = 0; k < 50; k++) {
        _pobj0[77].pRecentHistory[k] = false;
      }

      pTrackLogics[77] = &_pobj0[77];
      for (k = 0; k < 50; k++) {
        _pobj0[78].pRecentHistory[k] = false;
      }

      pTrackLogics[78] = &_pobj0[78];
      for (k = 0; k < 50; k++) {
        _pobj0[79].pRecentHistory[k] = false;
      }

      pTrackLogics[79] = &_pobj0[79];
      for (k = 0; k < 50; k++) {
        _pobj0[80].pRecentHistory[k] = false;
      }

      pTrackLogics[80] = &_pobj0[80];
      for (k = 0; k < 50; k++) {
        _pobj0[81].pRecentHistory[k] = false;
      }

      pTrackLogics[81] = &_pobj0[81];
      for (k = 0; k < 50; k++) {
        _pobj0[82].pRecentHistory[k] = false;
      }

      pTrackLogics[82] = &_pobj0[82];
      for (k = 0; k < 50; k++) {
        _pobj0[83].pRecentHistory[k] = false;
      }

      pTrackLogics[83] = &_pobj0[83];
      for (k = 0; k < 50; k++) {
        _pobj0[84].pRecentHistory[k] = false;
      }

      pTrackLogics[84] = &_pobj0[84];
      for (k = 0; k < 50; k++) {
        _pobj0[85].pRecentHistory[k] = false;
      }

      pTrackLogics[85] = &_pobj0[85];
      for (k = 0; k < 50; k++) {
        _pobj0[86].pRecentHistory[k] = false;
      }

      pTrackLogics[86] = &_pobj0[86];
      for (k = 0; k < 50; k++) {
        _pobj0[87].pRecentHistory[k] = false;
      }

      pTrackLogics[87] = &_pobj0[87];
      for (k = 0; k < 50; k++) {
        _pobj0[88].pRecentHistory[k] = false;
      }

      pTrackLogics[88] = &_pobj0[88];
      for (k = 0; k < 50; k++) {
        _pobj0[89].pRecentHistory[k] = false;
      }

      pTrackLogics[89] = &_pobj0[89];
      for (k = 0; k < 50; k++) {
        _pobj0[90].pRecentHistory[k] = false;
      }

      pTrackLogics[90] = &_pobj0[90];
      for (k = 0; k < 50; k++) {
        _pobj0[91].pRecentHistory[k] = false;
      }

      pTrackLogics[91] = &_pobj0[91];
      for (k = 0; k < 50; k++) {
        _pobj0[92].pRecentHistory[k] = false;
      }

      pTrackLogics[92] = &_pobj0[92];
      for (k = 0; k < 50; k++) {
        _pobj0[93].pRecentHistory[k] = false;
      }

      pTrackLogics[93] = &_pobj0[93];
      for (k = 0; k < 50; k++) {
        _pobj0[94].pRecentHistory[k] = false;
      }

      pTrackLogics[94] = &_pobj0[94];
      for (k = 0; k < 50; k++) {
        _pobj0[95].pRecentHistory[k] = false;
      }

      pTrackLogics[95] = &_pobj0[95];
      for (k = 0; k < 50; k++) {
        _pobj0[96].pRecentHistory[k] = false;
      }

      pTrackLogics[96] = &_pobj0[96];
      for (k = 0; k < 50; k++) {
        _pobj0[97].pRecentHistory[k] = false;
      }

      pTrackLogics[97] = &_pobj0[97];
      for (k = 0; k < 50; k++) {
        _pobj0[98].pRecentHistory[k] = false;
      }

      pTrackLogics[98] = &_pobj0[98];
      for (k = 0; k < 50; k++) {
        _pobj0[99].pRecentHistory[k] = false;
      }

      pTrackLogics[99] = &_pobj0[99];
      isSetupComplete = true;
      TunablePropsChanged = false;
      resetImpl();
    }

    if (TunablePropsChanged) {
      TunablePropsChanged = false;
    }

    inSize[0] = 1U;
    inSize[1] = static_cast<unsigned int>(varargin_1.size(1));
    for (k = 0; k < 6; k++) {
      inSize[k + 2] = 1U;
    }

    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 8)) {
      if (inputVarSize[0].f1[k] != inSize[k]) {
        for (k = 0; k < 8; k++) {
          inputVarSize[0].f1[k] = inSize[k];
        }

        exitg1 = true;
      } else {
        k++;
      }
    }

    return stepImpl(varargin_1, varargin_2, varargout_1_data, varargout_2_data,
                    varargout_3, varargout_4_TrackIDsAtStepBeginning_data,
                    varargout_4_TrackIDsAtStepBeginning_size,
                    varargout_4_CostMatrix, varargout_4_Assignments,
                    varargout_4_UnassignedCentralTracks,
                    varargout_4_UnassignedLocalTracks,
                    varargout_4_NonInitializingLocalTracks,
                    varargout_4_InitializedCentralTrackIDs_data,
                    varargout_4_InitializedCentralTrackIDs_size,
                    varargout_4_UpdatedCentralTrackIDs,
                    varargout_4_DeletedTrackIDs_data,
                    varargout_4_DeletedTrackIDs_size,
                    varargout_4_TrackIDsAtStepEnd_data,
                    varargout_4_TrackIDsAtStepEnd_size, varargout_2_size);
  }

  //
  // Arguments    : const array<struct_T, 2U> &localTracks
  //                double tFusion
  //                struct3_T confTracks_data[]
  //                struct3_T tentTracks_data[]
  //                array<struct3_T, 1U> &allTracks
  //                unsigned int info_TrackIDsAtStepBeginning_data[]
  //                int info_TrackIDsAtStepBeginning_size[2]
  //                array<double, 2U> &info_CostMatrix
  //                array<unsigned int, 2U> &info_Assignments
  //                array<unsigned int, 1U> &info_UnassignedCentralTracks
  //                array<unsigned int, 1U> &info_UnassignedLocalTracks
  //                array<unsigned int, 1U> &info_NonInitializingLocalTracks
  //                unsigned int info_InitializedCentralTrackIDs_data[]
  //                int info_InitializedCentralTrackIDs_size[2]
  //                array<unsigned int, 2U> &info_UpdatedCentralTrackIDs
  //                unsigned int info_DeletedTrackIDs_data[]
  //                int info_DeletedTrackIDs_size[2]
  //                unsigned int info_TrackIDsAtStepEnd_data[]
  //                int info_TrackIDsAtStepEnd_size[2]
  //                int &tentTracks_size
  // Return Type  : int
  //
  int trackFuser::stepImpl(const array<struct_T, 2U> &localTracks, double
    tFusion, struct3_T confTracks_data[], struct3_T tentTracks_data[], array<
    struct3_T, 1U> &allTracks, unsigned int info_TrackIDsAtStepBeginning_data[],
    int info_TrackIDsAtStepBeginning_size[2], array<double, 2U> &info_CostMatrix,
    array<unsigned int, 2U> &info_Assignments, array<unsigned int, 1U>
    &info_UnassignedCentralTracks, array<unsigned int, 1U>
    &info_UnassignedLocalTracks, array<unsigned int, 1U>
    &info_NonInitializingLocalTracks, unsigned int
    info_InitializedCentralTrackIDs_data[], int
    info_InitializedCentralTrackIDs_size[2], array<unsigned int, 2U>
    &info_UpdatedCentralTrackIDs, unsigned int info_DeletedTrackIDs_data[], int
    info_DeletedTrackIDs_size[2], unsigned int info_TrackIDsAtStepEnd_data[],
    int info_TrackIDsAtStepEnd_size[2], int &tentTracks_size)
  {
    static const char logicType[7]{ 'H', 'i', 's', 't', 'o', 'r', 'y' };

    fuserSourceConfiguration *source;
    fuserSourceConfiguration *thisSource;
    objectTrack obj;
    objectTrack track;
    trackHistoryLogic *b_obj;
    array<b_struct_T, 1U> allStructs;
    array<double, 2U> b_times;
    array<double, 2U> times;
    array<unsigned int, 2U> assigned;
    array<int, 2U> r1;
    array<unsigned int, 1U> assignedTrIDs;
    array<unsigned int, 1U> b_info_UnassignedLocalTracks;
    array<int, 1U> r;
    array<unsigned int, 1U> unassignedCentral;
    array<boolean_T, 2U> updated;
    array<boolean_T, 1U> isInitializing;
    b_struct_T oneStruct;
    double d;
    double prevNumLive;
    int b_end_tmp;
    int confTracks_size;
    int end_tmp;
    int i;
    int loop_ub;
    int loop_ub_tmp;
    int partialTrueCount;
    int trueCount;
    signed char b_tmp_data[100];
    signed char c_tmp_data[100];
    signed char d_tmp_data[100];
    signed char tmp_data[20];
    boolean_T deleted[100];
    boolean_T inKnownIDs[20];
    boolean_T exitg1;
    prevNumLive = pNumLiveTracks;
    if (std::isnan(prevNumLive)) {
      times.set_size(1, 1);
      times[0] = rtNaN;
    } else if (prevNumLive < 1.0) {
      times.set_size(1, 0);
    } else {
      times.set_size(1, static_cast<int>(prevNumLive - 1.0) + 1);
      loop_ub = static_cast<int>(prevNumLive - 1.0);
      for (i = 0; i <= loop_ub; i++) {
        times[i] = static_cast<double>(i) + 1.0;
      }
    }

    if (prevNumLive < 1.0) {
      loop_ub_tmp = 0;
    } else {
      loop_ub_tmp = static_cast<int>(prevNumLive);
    }

    info_TrackIDsAtStepBeginning_size[0] = 1;
    info_TrackIDsAtStepBeginning_size[1] = loop_ub_tmp;
    for (i = 0; i < loop_ub_tmp; i++) {
      info_TrackIDsAtStepBeginning_data[i] = pTrackIDs[i];
    }

    pUsedConfigIDs.set_size(localTracks.size(1), 20);
    loop_ub = localTracks.size(1) * 20;
    for (i = 0; i < loop_ub; i++) {
      pUsedConfigIDs[i] = false;
    }

    i = localTracks.size(1);
    for (int b_i{0}; b_i < i; b_i++) {
      boolean_T y;
      for (end_tmp = 0; end_tmp < 20; end_tmp++) {
        inKnownIDs[end_tmp] = (localTracks[b_i].SourceIndex ==
          pSourceConfigIDs[end_tmp]);
      }

      trueCount = 0;
      partialTrueCount = 0;
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        if (inKnownIDs[loop_ub]) {
          trueCount++;
          tmp_data[partialTrueCount] = static_cast<signed char>(loop_ub);
          partialTrueCount++;
        }
      }

      for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
        pUsedConfigIDs[b_i + pUsedConfigIDs.size(0) * tmp_data[end_tmp]] = true;
      }

      for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
        inKnownIDs[end_tmp] = !pIsValidSource[tmp_data[end_tmp]];
      }

      y = (trueCount != 0);
      if (y) {
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= trueCount - 1)) {
          if (!inKnownIDs[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }

      if (y) {
        for (end_tmp = 0; end_tmp < 20; end_tmp++) {
          inKnownIDs[end_tmp] = (localTracks[b_i].SourceIndex ==
            pSourceConfigIDs[end_tmp]);
        }

        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub < 20)) {
          if (inKnownIDs[loop_ub]) {
            source = pSourceConfigurations[loop_ub];
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }

        if (!source->pIsTransformToCentralValid) {
          source->pIsTransformToCentralValid = true;
        }

        for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
          pIsValidSource[tmp_data[end_tmp]] = true;
        }
      }
    }

    distance(localTracks, info_CostMatrix);
    assign(info_CostMatrix, assigned, unassignedCentral,
           info_UnassignedLocalTracks);
    assignedTrIDs.set_size(assigned.size(0));
    loop_ub = assigned.size(0);
    for (i = 0; i < loop_ub; i++) {
      assignedTrIDs[i] = pTrackIDs[static_cast<int>(assigned[i]) - 1];
    }

    info_UnassignedCentralTracks.set_size(unassignedCentral.size(0));
    loop_ub = unassignedCentral.size(0);
    for (i = 0; i < loop_ub; i++) {
      info_UnassignedCentralTracks[i] = pTrackIDs[static_cast<int>
        (unassignedCentral[i]) - 1];
    }

    isInitializing.set_size(info_UnassignedLocalTracks.size(0));
    i = info_UnassignedLocalTracks.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      for (end_tmp = 0; end_tmp < 20; end_tmp++) {
        inKnownIDs[end_tmp] = (localTracks[static_cast<int>
          (info_UnassignedLocalTracks[b_i]) - 1].SourceIndex ==
          pSourceConfigIDs[end_tmp]);
      }

      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 20)) {
        if (inKnownIDs[loop_ub]) {
          thisSource = pSourceConfigurations[loop_ub];
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }

      isInitializing[b_i] = thisSource->IsInitializingCentralTracks;
    }

    b_end_tmp = isInitializing.size(0) - 1;
    trueCount = 0;
    for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
      if (isInitializing[b_i]) {
        trueCount++;
      }
    }

    r.set_size(trueCount);
    partialTrueCount = 0;
    for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
      if (isInitializing[b_i]) {
        r[partialTrueCount] = b_i;
        partialTrueCount++;
      }
    }

    b_info_UnassignedLocalTracks.set_size(r.size(0));
    loop_ub = r.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_info_UnassignedLocalTracks[i] = info_UnassignedLocalTracks[r[i]];
    }

    initializeCentralTracks(localTracks, b_info_UnassignedLocalTracks);
    d = pNumLiveTracks;
    if (prevNumLive + 1.0 > d) {
      i = 0;
      end_tmp = 0;
    } else {
      i = static_cast<int>(prevNumLive + 1.0) - 1;
      end_tmp = static_cast<int>(d);
    }

    info_InitializedCentralTrackIDs_size[0] = 1;
    loop_ub = end_tmp - i;
    info_InitializedCentralTrackIDs_size[1] = loop_ub;
    for (end_tmp = 0; end_tmp < loop_ub; end_tmp++) {
      info_InitializedCentralTrackIDs_data[end_tmp] = pTrackIDs[i + end_tmp];
    }

    b_fuseAssigned(localTracks, assigned, updated);
    end_tmp = updated.size(1) - 1;
    trueCount = 0;
    for (int b_i{0}; b_i <= end_tmp; b_i++) {
      if (!updated[b_i]) {
        trueCount++;
      }
    }

    r1.set_size(1, trueCount);
    partialTrueCount = 0;
    for (int b_i{0}; b_i <= end_tmp; b_i++) {
      if (!updated[b_i]) {
        r1[partialTrueCount] = b_i;
        partialTrueCount++;
      }
    }

    b_times.set_size(1, r1.size(1));
    loop_ub = r1.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_times[i] = times[r1[i]];
    }

    coastUnassigned(unassignedCentral, b_times, deleted);
    d = pNumLiveTracks;
    i = static_cast<int>(d);
    for (int b_i{0}; b_i < i; b_i++) {
      track = pTracksList[b_i];
      prevNumLive = tFusion - track.pUpdateTime;
      if (prevNumLive > 0.0) {
        double x[6];
        track = pTracksList[b_i];
        obj = pTracksList[b_i];
        fusion::internal::gaussEKFilter::predict(track.pState,
          obj.pStateCovariance, ProcessNoise, prevNumLive, x);
        track = pTracksList[b_i];
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          track.pState[loop_ub] = x[loop_ub];
        }

        std::copy(&obj.pStateCovariance[0], &obj.pStateCovariance[36],
                  &track.pStateCovariance[0]);
        track.pUpdateTime = tFusion;
        pTracksList[b_i] = track;
      }
    }

    pLastTimeStamp = tFusion;
    d = pNumLiveTracks;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }

    info_TrackIDsAtStepEnd_size[0] = 1;
    info_TrackIDsAtStepEnd_size[1] = loop_ub;
    for (i = 0; i < loop_ub; i++) {
      info_TrackIDsAtStepEnd_data[i] = pTrackIDs[i];
    }

    if (info_UnassignedLocalTracks.size(0) == 0) {
      info_NonInitializingLocalTracks.set_size(0);
    } else {
      trueCount = 0;
      for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
        if (!isInitializing[b_i]) {
          trueCount++;
        }
      }

      info_NonInitializingLocalTracks.set_size(trueCount);
      partialTrueCount = 0;
      for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
        if (!isInitializing[b_i]) {
          info_NonInitializingLocalTracks[partialTrueCount] =
            info_UnassignedLocalTracks[b_i];
          partialTrueCount++;
        }
      }
    }

    info_Assignments.set_size(assignedTrIDs.size(0), 2);
    loop_ub = assignedTrIDs.size(0);
    for (i = 0; i < loop_ub; i++) {
      info_Assignments[i] = assignedTrIDs[i];
      info_Assignments[i + info_Assignments.size(0)] = assigned[i +
        assigned.size(0)];
    }

    trueCount = 0;
    for (int b_i{0}; b_i <= end_tmp; b_i++) {
      if (updated[b_i]) {
        trueCount++;
      }
    }

    info_UpdatedCentralTrackIDs.set_size(1, trueCount);
    partialTrueCount = 0;
    for (int b_i{0}; b_i <= end_tmp; b_i++) {
      if (updated[b_i]) {
        info_UpdatedCentralTrackIDs[partialTrueCount] =
          info_TrackIDsAtStepBeginning_data[b_i];
        partialTrueCount++;
      }
    }

    loop_ub = loop_ub_tmp - 1;
    trueCount = 0;
    partialTrueCount = 0;
    for (int b_i{0}; b_i <= loop_ub; b_i++) {
      if (deleted[b_i]) {
        trueCount++;
        b_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
        partialTrueCount++;
      }
    }

    info_DeletedTrackIDs_size[0] = 1;
    info_DeletedTrackIDs_size[1] = trueCount;
    for (i = 0; i < trueCount; i++) {
      info_DeletedTrackIDs_data[i] =
        info_TrackIDsAtStepBeginning_data[b_tmp_data[i]];
    }

    prevNumLive = pNumLiveTracks;
    if (std::isnan(prevNumLive)) {
      times.set_size(1, 1);
      times[0] = rtNaN;
    } else if (prevNumLive < 1.0) {
      times.set_size(1, 0);
    } else {
      times.set_size(1, static_cast<int>(prevNumLive - 1.0) + 1);
      loop_ub = static_cast<int>(prevNumLive - 1.0);
      for (i = 0; i <= loop_ub; i++) {
        times[i] = static_cast<double>(i) + 1.0;
      }
    }

    if (times.size(1) > 0) {
      loop_ub = static_cast<int>(times[0]) - 1;
      track = pTracksList[loop_ub];
      oneStruct.TrackID = track.TrackID;
      oneStruct.BranchID = track.BranchID;
      oneStruct.SourceIndex = track.SourceIndex;
      oneStruct.UpdateTime = track.pUpdateTime;
      oneStruct.Age = track.Age;
      for (int b_i{0}; b_i < 6; b_i++) {
        oneStruct.State[b_i] = track.pState[b_i];
      }

      std::copy(&track.pStateCovariance[0], &track.pStateCovariance[36],
                &oneStruct.StateCovariance[0]);
      oneStruct.ObjectClassID = track.ObjectClassID;
      oneStruct.ObjectClassProbabilities = track.ObjectClassProbabilities;
      for (i = 0; i < 7; i++) {
        oneStruct.TrackLogic[i] = logicType[i];
      }

      b_obj = pTrackLogics[loop_ub];
      for (i = 0; i < 5; i++) {
        oneStruct.TrackLogicState[i] = b_obj->pRecentHistory[i];
      }

      oneStruct.IsConfirmed = track.IsConfirmed;
      oneStruct.IsCoasted = track.IsCoasted;
      oneStruct.IsSelfReported = track.IsSelfReported;
      allStructs.set_size(times.size(1));
      loop_ub = times.size(1);
      for (i = 0; i < loop_ub; i++) {
        allStructs[i] = oneStruct;
      }

      i = times.size(1);
      for (int b_i{0}; b_i <= i - 2; b_i++) {
        d = times[b_i + 1];
        allStructs[b_i + 1].TrackID = pTracksList[static_cast<int>(d) - 1].
          TrackID;
        allStructs[b_i + 1].BranchID = pTracksList[static_cast<int>(d) - 1].
          BranchID;
        allStructs[b_i + 1].Age = pTracksList[static_cast<int>(d) - 1].Age;
        track = pTracksList[static_cast<int>(d) - 1];
        for (end_tmp = 0; end_tmp < 6; end_tmp++) {
          allStructs[b_i + 1].State[end_tmp] = track.pState[end_tmp];
        }

        track = pTracksList[static_cast<int>(d) - 1];
        for (end_tmp = 0; end_tmp < 36; end_tmp++) {
          allStructs[b_i + 1].StateCovariance[end_tmp] =
            track.pStateCovariance[end_tmp];
        }

        allStructs[b_i + 1].ObjectClassID = pTracksList[static_cast<int>(d) - 1]
          .ObjectClassID;
        b_obj = pTrackLogics[static_cast<int>(d) - 1];
        for (end_tmp = 0; end_tmp < 5; end_tmp++) {
          allStructs[b_i + 1].TrackLogicState[end_tmp] = b_obj->
            pRecentHistory[end_tmp];
        }

        allStructs[b_i + 1].IsConfirmed = pTracksList[static_cast<int>(d) - 1].
          IsConfirmed;
        allStructs[b_i + 1].IsCoasted = pTracksList[static_cast<int>(d) - 1].
          IsCoasted;
        allStructs[b_i + 1].IsSelfReported = pTracksList[static_cast<int>(d) - 1]
          .IsSelfReported;
      }

      allTracks.set_size(allStructs.size(0));
      i = allStructs.size(0);
      for (end_tmp = 0; end_tmp < i; end_tmp++) {
        allTracks[end_tmp].TrackID = allStructs[end_tmp].TrackID;
        allTracks[end_tmp].BranchID = allStructs[end_tmp].BranchID;
        allTracks[end_tmp].SourceIndex = allStructs[end_tmp].SourceIndex;
        allTracks[end_tmp].UpdateTime = allStructs[end_tmp].UpdateTime;
        allTracks[end_tmp].Age = allStructs[end_tmp].Age;
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          allTracks[end_tmp].State[loop_ub] = allStructs[end_tmp].State[loop_ub];
        }

        for (loop_ub = 0; loop_ub < 36; loop_ub++) {
          allTracks[end_tmp].StateCovariance[loop_ub] = allStructs[end_tmp].
            StateCovariance[loop_ub];
        }

        allTracks[end_tmp].ObjectClassID = allStructs[end_tmp].ObjectClassID;
        allTracks[end_tmp].ObjectClassProbabilities = allStructs[end_tmp].
          ObjectClassProbabilities;
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          allTracks[end_tmp].TrackLogic[loop_ub] = allStructs[end_tmp].
            TrackLogic[loop_ub];
        }

        allTracks[end_tmp].TrackLogicState.set_size(1, 5);
        for (loop_ub = 0; loop_ub < 5; loop_ub++) {
          allTracks[end_tmp].TrackLogicState[loop_ub] = allStructs[end_tmp].
            TrackLogicState[loop_ub];
        }

        allTracks[end_tmp].IsConfirmed = allStructs[end_tmp].IsConfirmed;
        allTracks[end_tmp].IsCoasted = allStructs[end_tmp].IsCoasted;
        allTracks[end_tmp].IsSelfReported = allStructs[end_tmp].IsSelfReported;
      }
    } else {
      allTracks.set_size(0);
    }

    confTracks_size = 0;
    for (int b_i{0}; b_i < 100; b_i++) {
      if (pConfirmedTracks[b_i]) {
        confTracks_size++;
      }
    }

    partialTrueCount = 0;
    for (int b_i{0}; b_i < 100; b_i++) {
      if (pConfirmedTracks[b_i]) {
        c_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
        partialTrueCount++;
      }
    }

    for (i = 0; i < confTracks_size; i++) {
      confTracks_data[i] = allTracks[static_cast<int>(c_tmp_data[i])];
    }

    d = pNumLiveTracks;
    if (d < 1.0) {
      i = 0;
    } else {
      i = static_cast<int>(d);
    }

    loop_ub = i - 1;
    tentTracks_size = 0;
    for (int b_i{0}; b_i <= loop_ub; b_i++) {
      if (!pConfirmedTracks[b_i]) {
        tentTracks_size++;
      }
    }

    partialTrueCount = 0;
    for (int b_i{0}; b_i <= loop_ub; b_i++) {
      if (!pConfirmedTracks[b_i]) {
        d_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
        partialTrueCount++;
      }
    }

    for (i = 0; i < tentTracks_size; i++) {
      tentTracks_data[i] = allTracks[static_cast<int>(d_tmp_data[i])];
    }

    return confTracks_size;
  }

  //
  // Arguments    : coder::array<double, 2U> &in1
  //                int in2
  //                const coder::array<double, 2U> &in3
  // Return Type  : void
  //
}

static void binary_expand_op(coder::array<double, 2U> &in1, int in2, const coder::
  array<double, 2U> &in3)
{
  coder::array<double, 2U> b_in1;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  i = in2 << 1;
  if (in3.size(1) == 1) {
    loop_ub = in1.size(1);
  } else {
    loop_ub = in3.size(1);
  }

  b_in1.set_size(1, loop_ub);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_1 = (in3.size(1) != 1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    b_in1[i1] = in1[(i + 6 * (i1 * stride_0_1)) - 1] + in3[i1 * stride_1_1];
  }

  loop_ub = b_in1.size(1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    in1[(i + 6 * i1) - 1] = b_in1[i1];
  }
}

//
// Arguments    : coder::array<double, 2U> &in1
//                int in2
//                int in3
//                const coder::array<double, 2U> &in4
//                const coder::array<double, 2U> &in5
// Return Type  : void
//
static void binary_expand_op_1(coder::array<double, 2U> &in1, int in2, int in3,
  const coder::array<double, 2U> &in4, const coder::array<double, 2U> &in5)
{
  coder::array<double, 2U> b_in1;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  i = in2 << 1;
  if (in5.size(1) == 1) {
    if (in4.size(1) == 1) {
      loop_ub = in1.size(1);
    } else {
      loop_ub = in4.size(1);
    }
  } else {
    loop_ub = in5.size(1);
  }

  b_in1.set_size(1, loop_ub);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_1 = (in4.size(1) != 1);
  stride_2_1 = (in5.size(1) != 1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    b_in1[i1] = (in1[in3 + 6 * (i1 * stride_0_1)] + in4[i1 * stride_1_1]) +
      in5[i1 * stride_2_1];
  }

  loop_ub = b_in1.size(1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    in1[(i + 6 * i1) - 2] = b_in1[i1];
  }
}

//
// Arguments    : const array<unsigned int, 1U> &a
//                const array<unsigned int, 1U> &b
//                array<unsigned int, 1U> &c
//                array<int, 1U> &ia
//                array<int, 1U> &ib
// Return Type  : void
//
namespace coder
{
  static void do_vectors(const array<unsigned int, 1U> &a, const array<unsigned
    int, 1U> &b, array<unsigned int, 1U> &c, array<int, 1U> &ia, array<int, 1U>
    &ib)
  {
    int iafirst;
    int ialast;
    int ibfirst;
    int iblast;
    int nc;
    int ncmax;
    nc = a.size(0);
    ncmax = b.size(0);
    if (nc <= ncmax) {
      ncmax = nc;
    }

    c.set_size(ncmax);
    ia.set_size(ncmax);
    ib.set_size(ncmax);
    nc = 0;
    iafirst = 0;
    ialast = 1;
    ibfirst = 0;
    iblast = 1;
    while ((ialast <= a.size(0)) && (iblast <= b.size(0))) {
      unsigned int ak;
      int b_ialast;
      int b_iblast;
      unsigned int bk;
      b_ialast = ialast;
      ak = a[ialast - 1];
      while ((b_ialast < a.size(0)) && (a[b_ialast] == ak)) {
        b_ialast++;
      }

      ialast = b_ialast;
      b_iblast = iblast;
      bk = b[iblast - 1];
      while ((b_iblast < b.size(0)) && (b[b_iblast] == bk)) {
        b_iblast++;
      }

      iblast = b_iblast;
      if (ak == bk) {
        nc++;
        c[nc - 1] = ak;
        ia[nc - 1] = iafirst + 1;
        ib[nc - 1] = ibfirst + 1;
        ialast = b_ialast + 1;
        iafirst = b_ialast;
        iblast = b_iblast + 1;
        ibfirst = b_iblast;
      } else if (ak < bk) {
        ialast = b_ialast + 1;
        iafirst = b_ialast;
      } else {
        iblast = b_iblast + 1;
        ibfirst = b_iblast;
      }
    }

    if (ncmax > 0) {
      if (nc < 1) {
        nc = 0;
      }

      ia.set_size(nc);
      ib.set_size(nc);
      c.set_size(nc);
    }
  }

  //
  // Arguments    : const double A[36]
  //                creal_T V[36]
  //                creal_T D[36]
  // Return Type  : void
  //
  static void eigStandard(const double A[36], creal_T V[36], creal_T D[36])
  {
    creal_T W[6];
    double b_A[36];
    double vr[36];
    double absxk;
    double anrm;
    int ihi;
    int k;
    boolean_T exitg1;
    std::copy(&A[0], &A[36], &b_A[0]);
    anrm = 0.0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 36)) {
      absxk = std::abs(A[k]);
      if (std::isnan(absxk)) {
        anrm = rtNaN;
        exitg1 = true;
      } else {
        if (absxk > anrm) {
          anrm = absxk;
        }

        k++;
      }
    }

    if (std::isinf(anrm) || std::isnan(anrm)) {
      for (int i{0}; i < 6; i++) {
        W[i].re = rtNaN;
        W[i].im = 0.0;
      }

      for (int b_i{0}; b_i < 36; b_i++) {
        V[b_i].re = rtNaN;
        V[b_i].im = 0.0;
      }
    } else {
      double scale[6];
      double wi[6];
      double wr[6];
      double tau[5];
      double cscale;
      int ilo;
      int info;
      boolean_T scalea;
      cscale = anrm;
      scalea = false;
      if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
        scalea = true;
        cscale = 6.7178761075670888E-139;
        internal::reflapack::xzlascl(anrm, cscale, b_A);
      } else if (anrm > 1.4885657073574029E+138) {
        scalea = true;
        cscale = 1.4885657073574029E+138;
        internal::reflapack::xzlascl(anrm, cscale, b_A);
      }

      ilo = internal::reflapack::xzgebal(b_A, ihi, scale);
      internal::reflapack::xzgehrd(b_A, ilo, ihi, tau);
      std::copy(&b_A[0], &b_A[36], &vr[0]);
      internal::reflapack::xzunghr(ilo, ihi, vr, tau);
      info = internal::reflapack::xdlahqr(ilo, ihi, b_A, ilo, ihi, vr, wr, wi);
      if (info == 0) {
        double f1;
        int b_i;
        int temp_tmp;
        int vr_tmp;
        internal::reflapack::xdtrevc3(b_A, vr);
        if (ilo != ihi) {
          for (int i{ilo}; i <= ihi; i++) {
            b_i = i + 30;
            for (k = i; k <= b_i; k += 6) {
              vr[k - 1] *= scale[i - 1];
            }
          }
        }

        b_i = ilo - 1;
        for (int i{b_i}; i >= 1; i--) {
          f1 = scale[i - 1];
          if (static_cast<int>(f1) != i) {
            for (k = 0; k < 6; k++) {
              temp_tmp = (i + k * 6) - 1;
              absxk = vr[temp_tmp];
              vr_tmp = (static_cast<int>(f1) + k * 6) - 1;
              vr[temp_tmp] = vr[vr_tmp];
              vr[vr_tmp] = absxk;
            }
          }
        }

        b_i = ihi + 1;
        for (int i{b_i}; i < 7; i++) {
          f1 = scale[i - 1];
          if (static_cast<int>(f1) != i) {
            for (k = 0; k < 6; k++) {
              temp_tmp = (i + k * 6) - 1;
              absxk = vr[temp_tmp];
              vr_tmp = (static_cast<int>(f1) + k * 6) - 1;
              vr[temp_tmp] = vr[vr_tmp];
              vr[vr_tmp] = absxk;
            }
          }
        }

        for (int i{0}; i < 6; i++) {
          f1 = wi[i];
          if (!(f1 < 0.0)) {
            if ((i + 1 != 6) && (f1 > 0.0)) {
              double c;
              double f1_tmp;
              double g1_tmp;
              double s;
              int ix0_tmp;
              int scl_tmp;
              scl_tmp = (i + 1) * 6;
              absxk = 1.0 / rt_hypotd_snf(internal::blas::xnrm2(6, vr, i * 6 + 1),
                internal::blas::xnrm2(6, vr, scl_tmp + 1));
              ix0_tmp = i * 6;
              b_i = ix0_tmp + 6;
              for (k = ix0_tmp + 1; k <= b_i; k++) {
                vr[k - 1] *= absxk;
              }

              b_i = scl_tmp + 6;
              for (k = scl_tmp + 1; k <= b_i; k++) {
                vr[k - 1] *= absxk;
              }

              for (vr_tmp = 0; vr_tmp < 6; vr_tmp++) {
                f1 = vr[vr_tmp + 6 * i];
                absxk = vr[vr_tmp + scl_tmp];
                scale[vr_tmp] = f1 * f1 + absxk * absxk;
              }

              k = 0;
              absxk = std::abs(scale[0]);
              for (ihi = 0; ihi < 5; ihi++) {
                s = std::abs(scale[ihi + 1]);
                if (s > absxk) {
                  k = ihi + 1;
                  absxk = s;
                }
              }

              f1_tmp = vr[k + 6 * i];
              f1 = std::abs(f1_tmp);
              ihi = k + scl_tmp;
              g1_tmp = vr[ihi];
              absxk = std::abs(g1_tmp);
              if (g1_tmp == 0.0) {
                c = 1.0;
                s = 0.0;
              } else if (f1_tmp == 0.0) {
                c = 0.0;
                if (g1_tmp >= 0.0) {
                  s = 1.0;
                } else {
                  s = -1.0;
                }
              } else if ((f1 > 1.4916681462400413E-154) && (f1 <
                          4.7403759540545887E+153) && (absxk >
                          1.4916681462400413E-154) && (absxk <
                          4.7403759540545887E+153)) {
                double d;
                d = std::sqrt(f1_tmp * f1_tmp + g1_tmp * g1_tmp);
                c = f1 / d;
                if (!(f1_tmp >= 0.0)) {
                  d = -d;
                }

                s = g1_tmp / d;
              } else {
                double d;
                absxk = std::fmin(4.49423283715579E+307, std::fmax
                                  (2.2250738585072014E-308, std::fmax(f1, absxk)));
                s = f1_tmp / absxk;
                absxk = g1_tmp / absxk;
                d = std::sqrt(s * s + absxk * absxk);
                c = std::abs(s) / d;
                if (!(f1_tmp >= 0.0)) {
                  d = -d;
                }

                s = absxk / d;
              }

              for (k = 0; k < 6; k++) {
                temp_tmp = scl_tmp + k;
                absxk = vr[temp_tmp];
                vr_tmp = ix0_tmp + k;
                f1 = vr[vr_tmp];
                vr[temp_tmp] = c * absxk - s * f1;
                vr[vr_tmp] = c * f1 + s * absxk;
              }

              vr[ihi] = 0.0;
            } else {
              absxk = 1.0 / internal::blas::xnrm2(6, vr, i * 6 + 1);
              ihi = i * 6;
              b_i = ihi + 6;
              for (k = ihi + 1; k <= b_i; k++) {
                vr[k - 1] *= absxk;
              }
            }
          }
        }

        for (b_i = 0; b_i < 36; b_i++) {
          V[b_i].re = vr[b_i];
          V[b_i].im = 0.0;
        }

        for (vr_tmp = 0; vr_tmp < 5; vr_tmp++) {
          if ((wi[vr_tmp] > 0.0) && (wi[vr_tmp + 1] < 0.0)) {
            for (int i{0}; i < 6; i++) {
              ihi = i + 6 * vr_tmp;
              temp_tmp = i + 6 * (vr_tmp + 1);
              absxk = V[temp_tmp].re;
              V[ihi].im = absxk;
              V[temp_tmp].re = V[ihi].re;
              V[temp_tmp].im = -absxk;
            }
          }
        }
      } else {
        for (int b_i{0}; b_i < 36; b_i++) {
          V[b_i].re = rtNaN;
          V[b_i].im = 0.0;
        }
      }

      if (scalea) {
        internal::reflapack::xzlascl(cscale, anrm, 6 - info, wr, info + 1);
        internal::reflapack::xzlascl(cscale, anrm, 6 - info, wi, info + 1);
        if (info != 0) {
          internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wr, 1);
          internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wi, 1);
        }
      }

      if (info != 0) {
        for (int i{ilo}; i <= info; i++) {
          wr[i - 1] = rtNaN;
          wi[i - 1] = 0.0;
        }
      }

      for (int i{0}; i < 6; i++) {
        W[i].re = wr[i];
        W[i].im = wi[i];
      }
    }

    std::memset(&D[0], 0, 36U * sizeof(creal_T));
    for (k = 0; k < 6; k++) {
      D[k + 6 * k] = W[k];
    }
  }

  //
  // Arguments    : const array<boolean_T, 1U> &x
  //                array<int, 1U> &i
  // Return Type  : void
  //
  static void eml_find(const array<boolean_T, 1U> &x, array<int, 1U> &i)
  {
    int idx;
    int ii;
    int nx;
    boolean_T exitg1;
    nx = x.size(0);
    idx = 0;
    i.set_size(x.size(0));
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii <= nx - 1)) {
      if (x[ii]) {
        idx++;
        i[idx - 1] = ii + 1;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          ii++;
        }
      } else {
        ii++;
      }
    }

    if (x.size(0) == 1) {
      if (idx == 0) {
        i.set_size(0);
      }
    } else {
      if (idx < 1) {
        idx = 0;
      }

      i.set_size(idx);
    }
  }

  //
  // Arguments    : double P[36]
  // Return Type  : void
  //
  namespace fusion
  {
    namespace internal
    {
      static void ensurePosDefMatrix(double P[36])
      {
        creal_T D[36];
        creal_T V[36];
        creal_T b_V[36];
        double A[36];
        double b_D[36];
        double work[6];
        double e[5];
        double absx;
        double anrm;
        double taui;
        double temp1;
        double temp2;
        int i;
        int i1;
        int i2;
        int iaii;
        int k;
        int sgn;
        boolean_T iscale;
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            iaii = i1 + 6 * i;
            A[iaii] = (P[iaii] + P[i + 6 * i1]) / 2.0;
          }
        }

        iscale = true;
        for (k = 0; k < 36; k++) {
          if (iscale) {
            absx = A[k];
            if (std::isinf(absx) || std::isnan(absx)) {
              iscale = false;
            }
          } else {
            iscale = false;
          }
        }

        if (!iscale) {
          for (i = 0; i < 36; i++) {
            V[i].re = rtNaN;
            V[i].im = 0.0;
            D[i].re = 0.0;
            D[i].im = 0.0;
          }

          for (k = 0; k < 6; k++) {
            i = k + 6 * k;
            D[i].re = rtNaN;
            D[i].im = 0.0;
          }
        } else {
          int b_i;
          int exitg1;
          boolean_T exitg2;
          iscale = true;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k < 6)) {
            b_i = 0;
            do {
              exitg1 = 0;
              if (b_i <= k) {
                if (!(A[b_i + 6 * k] == A[k + 6 * b_i])) {
                  iscale = false;
                  exitg1 = 1;
                } else {
                  b_i++;
                }
              } else {
                k++;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          if (iscale) {
            double a__4[6];
            anrm = 0.0;
            k = 0;
            exitg2 = false;
            while ((!exitg2) && (k < 6)) {
              b_i = 0;
              do {
                exitg1 = 0;
                if (b_i <= k) {
                  absx = std::abs(A[b_i + 6 * k]);
                  if (std::isnan(absx)) {
                    anrm = rtNaN;
                    exitg1 = 1;
                  } else {
                    if (absx > anrm) {
                      anrm = absx;
                    }

                    b_i++;
                  }
                } else {
                  k++;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }

            if (std::isinf(anrm) || std::isnan(anrm)) {
              for (b_i = 0; b_i < 6; b_i++) {
                a__4[b_i] = rtNaN;
              }

              for (i = 0; i < 36; i++) {
                A[i] = rtNaN;
              }
            } else {
              double tau[5];
              iscale = false;
              if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
                iscale = true;
                anrm = 1.0010415475915505E-146 / anrm;
                ::coder::internal::reflapack::xzlascl(1.0, anrm, A);
              } else if (anrm > 9.9895953610111751E+145) {
                iscale = true;
                anrm = 9.9895953610111751E+145 / anrm;
                ::coder::internal::reflapack::xzlascl(1.0, anrm, A);
              }

              for (b_i = 0; b_i < 5; b_i++) {
                int e_tmp_tmp;
                e_tmp_tmp = b_i + 6 * b_i;
                e[b_i] = A[e_tmp_tmp + 1];
                sgn = b_i + 3;
                if (sgn > 6) {
                  sgn = 6;
                }

                taui = ::coder::internal::reflapack::xzlarfg(5 - b_i, e[b_i], A,
                  b_i * 6 + sgn);
                if (taui != 0.0) {
                  int tau_tmp;
                  A[e_tmp_tmp + 1] = 1.0;
                  for (sgn = b_i + 1; sgn < 6; sgn++) {
                    tau[sgn - 1] = 0.0;
                  }

                  i = 4 - b_i;
                  i1 = 5 - b_i;
                  for (int jj{0}; jj <= i; jj++) {
                    iaii = b_i + jj;
                    temp1 = taui * A[(iaii + 6 * b_i) + 1];
                    temp2 = 0.0;
                    tau_tmp = 6 * (iaii + 1);
                    tau[iaii] += temp1 * A[(iaii + tau_tmp) + 1];
                    i2 = jj + 2;
                    for (int ii{i2}; ii <= i1; ii++) {
                      sgn = b_i + ii;
                      absx = A[sgn + tau_tmp];
                      tau[sgn - 1] += temp1 * absx;
                      temp2 += absx * A[sgn + 6 * b_i];
                    }

                    tau[iaii] += taui * temp2;
                  }

                  temp2 = 0.0;
                  for (k = 0; k <= i; k++) {
                    temp2 += tau[b_i + k] * A[(e_tmp_tmp + k) + 1];
                  }

                  temp2 *= -0.5 * taui;
                  if (!(temp2 == 0.0)) {
                    for (k = 0; k <= i; k++) {
                      tau_tmp = b_i + k;
                      tau[tau_tmp] += temp2 * A[(e_tmp_tmp + k) + 1];
                    }
                  }

                  for (int jj{0}; jj <= i; jj++) {
                    iaii = b_i + jj;
                    temp1 = A[(iaii + 6 * b_i) + 1];
                    absx = tau[iaii];
                    temp2 = absx * temp1;
                    k = 6 * (iaii + 1);
                    iaii = (iaii + k) + 1;
                    A[iaii] = (A[iaii] - temp2) - temp2;
                    i2 = jj + 2;
                    for (int ii{i2}; ii <= i1; ii++) {
                      iaii = b_i + ii;
                      sgn = iaii + k;
                      A[sgn] = (A[sgn] - tau[iaii - 1] * temp1) - A[iaii + 6 *
                        b_i] * absx;
                    }
                  }
                }

                A[e_tmp_tmp + 1] = e[b_i];
                a__4[b_i] = A[e_tmp_tmp];
                tau[b_i] = taui;
              }

              a__4[5] = A[35];
              for (k = 4; k >= 0; k--) {
                iaii = 6 * (k + 1);
                A[iaii] = 0.0;
                i = k + 3;
                for (b_i = i; b_i < 7; b_i++) {
                  A[(b_i + iaii) - 1] = A[(b_i + 6 * k) - 1];
                }
              }

              A[0] = 1.0;
              for (b_i = 0; b_i < 5; b_i++) {
                A[b_i + 1] = 0.0;
              }

              for (b_i = 0; b_i < 6; b_i++) {
                work[b_i] = 0.0;
              }

              for (b_i = 4; b_i >= 0; b_i--) {
                iaii = (b_i + b_i * 6) + 7;
                if (b_i + 1 < 5) {
                  A[iaii] = 1.0;
                  ::coder::internal::reflapack::xzlarf(5 - b_i, 4 - b_i, iaii +
                    1, tau[b_i], A, iaii + 7, work);
                  sgn = iaii + 2;
                  i = (iaii - b_i) + 5;
                  for (k = sgn; k <= i; k++) {
                    A[k - 1] *= -tau[b_i];
                  }
                }

                A[iaii] = 1.0 - tau[b_i];
                for (k = 0; k < b_i; k++) {
                  A[(iaii - k) - 1] = 0.0;
                }
              }

              sgn = ::coder::internal::reflapack::xzsteqr(a__4, e, A);
              if (sgn != 0) {
                for (b_i = 0; b_i < 6; b_i++) {
                  a__4[b_i] = rtNaN;
                }

                for (i = 0; i < 36; i++) {
                  A[i] = rtNaN;
                }
              } else if (iscale) {
                temp2 = 1.0 / anrm;
                for (k = 0; k < 6; k++) {
                  a__4[k] *= temp2;
                }
              }
            }

            std::memset(&D[0], 0, 36U * sizeof(creal_T));
            for (b_i = 0; b_i < 6; b_i++) {
              i = b_i + 6 * b_i;
              D[i].re = a__4[b_i];
              D[i].im = 0.0;
            }

            for (i = 0; i < 36; i++) {
              V[i].re = A[i];
              V[i].im = 0.0;
            }
          } else {
            iscale = true;
            k = 0;
            exitg2 = false;
            while ((!exitg2) && (k < 6)) {
              b_i = 0;
              do {
                exitg1 = 0;
                if (b_i <= k) {
                  if (!(A[b_i + 6 * k] == -A[k + 6 * b_i])) {
                    iscale = false;
                    exitg1 = 1;
                  } else {
                    b_i++;
                  }
                } else {
                  k++;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }

            if (iscale) {
              double a__4[6];
              double tau[5];
              ::coder::internal::reflapack::xzgehrd(A, 1, 6, tau);
              std::copy(&A[0], &A[36], &b_D[0]);
              ::coder::internal::reflapack::xzunghr(1, 6, b_D, tau);
              sgn = ::coder::internal::reflapack::xdlahqr(1, 6, A, 1, 6, b_D,
                a__4, work);
              std::memset(&D[0], 0, 36U * sizeof(creal_T));
              i = static_cast<unsigned char>(sgn);
              for (b_i = 0; b_i < i; b_i++) {
                i1 = b_i + 6 * b_i;
                D[i1].re = rtNaN;
                D[i1].im = 0.0;
              }

              i = sgn + 1;
              for (b_i = i; b_i < 7; b_i++) {
                i1 = (b_i + 6 * (b_i - 1)) - 1;
                D[i1].re = 0.0;
                D[i1].im = work[b_i - 1];
              }

              if (sgn == 0) {
                for (i = 0; i < 36; i++) {
                  V[i].re = b_D[i];
                  V[i].im = 0.0;
                }

                k = 1;
                do {
                  exitg1 = 0;
                  if (k <= 6) {
                    if (k != 6) {
                      i = 6 * (k - 1);
                      absx = A[k + i];
                      if (absx != 0.0) {
                        if (absx < 0.0) {
                          sgn = 1;
                        } else {
                          sgn = -1;
                        }

                        for (b_i = 0; b_i < 6; b_i++) {
                          i1 = b_i + i;
                          absx = V[i1].re;
                          i2 = b_i + 6 * k;
                          temp2 = static_cast<double>(sgn) * V[i2].re;
                          if (temp2 == 0.0) {
                            V[i1].re = absx / 1.4142135623730951;
                            V[i1].im = 0.0;
                          } else if (absx == 0.0) {
                            V[i1].re = 0.0;
                            V[i1].im = temp2 / 1.4142135623730951;
                          } else {
                            V[i1].re = absx / 1.4142135623730951;
                            V[i1].im = temp2 / 1.4142135623730951;
                          }

                          V[i2].re = V[i1].re;
                          V[i2].im = -V[i1].im;
                        }

                        k += 2;
                      } else {
                        k++;
                      }
                    } else {
                      k++;
                    }
                  } else {
                    exitg1 = 1;
                  }
                } while (exitg1 == 0);
              } else {
                for (i = 0; i < 36; i++) {
                  V[i].re = rtNaN;
                  V[i].im = 0.0;
                }
              }
            } else {
              eigStandard(A, V, D);
            }
          }
        }

        for (k = 0; k < 6; k++) {
          work[k] = std::fmax(D[k + 6 * k].re, 2.2204460492503131E-16);
        }

        std::memset(&b_D[0], 0, 36U * sizeof(double));
        for (k = 0; k < 6; k++) {
          b_D[k + 6 * k] = work[k];
        }

        for (i = 0; i < 36; i++) {
          D[i].re = b_D[i];
          D[i].im = 0.0;
        }

        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            absx = 0.0;
            temp2 = 0.0;
            for (i2 = 0; i2 < 6; i2++) {
              double V_re_tmp;
              sgn = i + 6 * i2;
              temp1 = V[sgn].re;
              iaii = i2 + 6 * i1;
              taui = D[iaii].im;
              anrm = V[sgn].im;
              V_re_tmp = D[iaii].re;
              absx += temp1 * V_re_tmp - anrm * taui;
              temp2 += temp1 * taui + anrm * V_re_tmp;
            }

            i2 = i + 6 * i1;
            b_V[i2].re = absx;
            b_V[i2].im = temp2;
          }

          for (i1 = 0; i1 < 6; i1++) {
            absx = 0.0;
            for (i2 = 0; i2 < 6; i2++) {
              sgn = i1 + 6 * i2;
              iaii = i + 6 * i2;
              absx += b_V[iaii].re * V[sgn].re - b_V[iaii].im * -V[sgn].im;
            }

            P[i + 6 * i1] = absx;
          }
        }
      }

      //
      // Arguments    : int n
      //                double a
      //                const double x[36]
      //                int ix0
      //                double y[18]
      // Return Type  : void
      //
    }
  }

  namespace internal
  {
    namespace blas
    {
      static void xaxpy(int n, double a, const double x[36], int ix0, double y
                        [18])
      {
        if ((n >= 1) && (!(a == 0.0))) {
          int i;
          i = n - 1;
          for (int k{0}; k <= i; k++) {
            y[k + 12] += a * x[(ix0 + k) - 1];
          }
        }
      }

      //
      // Arguments    : int n
      //                const double x[18]
      //                double beta1
      //                double y[36]
      //                int iy0
      // Return Type  : void
      //
      static void xgemv(int n, const double x[18], double beta1, double y[36],
                        int iy0)
      {
        int iy;
        int iyend;
        iyend = iy0 + 5;
        if (beta1 != 1.0) {
          if (beta1 == 0.0) {
            if (iy0 <= iyend) {
              std::memset(&y[iy0 + -1], 0, static_cast<unsigned int>((iyend -
                iy0) + 1) * sizeof(double));
            }
          } else {
            for (iy = iy0; iy <= iyend; iy++) {
              y[iy - 1] *= beta1;
            }
          }
        }

        iyend = 12;
        iy = 6 * (n - 1) + 1;
        for (int iac{1}; iac <= iy; iac += 6) {
          int i;
          i = iac + 5;
          for (int ia{iac}; ia <= i; ia++) {
            int i1;
            i1 = ((iy0 + ia) - iac) - 1;
            y[i1] += y[ia - 1] * x[iyend];
          }

          iyend++;
        }
      }

      //
      // Arguments    : int n
      //                const array<double, 2U> &x
      //                int ix0
      // Return Type  : double
      //
      static double xnrm2(int n, const array<double, 2U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k{ix0}; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      //
      // Arguments    : int n
      //                const double x[36]
      //                int ix0
      // Return Type  : double
      //
      static double xnrm2(int n, const double x[36], int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k{ix0}; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      //
      // Arguments    : int n
      //                const double x[3]
      // Return Type  : double
      //
      static double xnrm2(int n, const double x[3])
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[1]);
          } else {
            double absxk;
            double scale;
            double t;
            scale = 3.3121686421112381E-170;
            absxk = std::abs(x[1]);
            if (absxk > 3.3121686421112381E-170) {
              y = 1.0;
              scale = absxk;
            } else {
              t = absxk / 3.3121686421112381E-170;
              y = t * t;
            }

            absxk = std::abs(x[2]);
            if (absxk > scale) {
              t = scale / absxk;
              y = y * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              y += t * t;
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      //
      // Arguments    : array<int, 2U> &idx
      //                array<double, 2U> &x
      //                int offset
      //                int np
      //                int nq
      //                array<int, 1U> &iwork
      //                array<double, 1U> &xwork
      // Return Type  : void
      //
    }

    static void merge(array<int, 2U> &idx, array<double, 2U> &x, int offset, int
                      np, int nq, array<int, 1U> &iwork, array<double, 1U>
                      &xwork)
    {
      if (nq != 0) {
        int iout;
        int n_tmp;
        int p;
        int q;
        n_tmp = np + nq;
        for (int j{0}; j < n_tmp; j++) {
          iout = offset + j;
          iwork[j] = idx[iout];
          xwork[j] = x[iout];
        }

        p = 0;
        q = np;
        iout = offset - 1;
        int exitg1;
        do {
          exitg1 = 0;
          iout++;
          if (xwork[p] <= xwork[q]) {
            idx[iout] = iwork[p];
            x[iout] = xwork[p];
            if (p + 1 < np) {
              p++;
            } else {
              exitg1 = 1;
            }
          } else {
            idx[iout] = iwork[q];
            x[iout] = xwork[q];
            if (q + 1 < n_tmp) {
              q++;
            } else {
              q = iout - p;
              for (int j{p + 1}; j <= np; j++) {
                iout = q + j;
                idx[iout] = iwork[j - 1];
                x[iout] = xwork[j - 1];
              }

              exitg1 = 1;
            }
          }
        } while (exitg1 == 0);
      }
    }

    //
    // Arguments    : array<int, 1U> &idx
    //                array<unsigned int, 1U> &x
    //                int offset
    //                int np
    //                int nq
    //                array<int, 1U> &iwork
    //                array<unsigned int, 1U> &xwork
    // Return Type  : void
    //
    static void merge(array<int, 1U> &idx, array<unsigned int, 1U> &x, int
                      offset, int np, int nq, array<int, 1U> &iwork, array<
                      unsigned int, 1U> &xwork)
    {
      if (nq != 0) {
        int iout;
        int n_tmp;
        int p;
        int q;
        n_tmp = np + nq;
        for (int j{0}; j < n_tmp; j++) {
          iout = offset + j;
          iwork[j] = idx[iout];
          xwork[j] = x[iout];
        }

        p = 0;
        q = np;
        iout = offset - 1;
        int exitg1;
        do {
          exitg1 = 0;
          iout++;
          if (xwork[p] <= xwork[q]) {
            idx[iout] = iwork[p];
            x[iout] = xwork[p];
            if (p + 1 < np) {
              p++;
            } else {
              exitg1 = 1;
            }
          } else {
            idx[iout] = iwork[q];
            x[iout] = xwork[q];
            if (q + 1 < n_tmp) {
              q++;
            } else {
              q = iout - p;
              for (int j{p + 1}; j <= np; j++) {
                iout = q + j;
                idx[iout] = iwork[j - 1];
                x[iout] = xwork[j - 1];
              }

              exitg1 = 1;
            }
          }
        } while (exitg1 == 0);
      }
    }

    //
    // Arguments    : array<int, 2U> &idx
    //                array<double, 2U> &x
    //                int offset
    //                int n
    //                int preSortLevel
    //                array<int, 1U> &iwork
    //                array<double, 1U> &xwork
    // Return Type  : void
    //
    static void merge_block(array<int, 2U> &idx, array<double, 2U> &x, int
      offset, int n, int preSortLevel, array<int, 1U> &iwork, array<double, 1U>
      &xwork)
    {
      int bLen;
      int nPairs;
      nPairs = n >> preSortLevel;
      bLen = 1 << preSortLevel;
      while (nPairs > 1) {
        int nTail;
        int tailOffset;
        if ((nPairs & 1) != 0) {
          nPairs--;
          tailOffset = bLen * nPairs;
          nTail = n - tailOffset;
          if (nTail > bLen) {
            merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
          }
        }

        tailOffset = bLen << 1;
        nPairs >>= 1;
        for (nTail = 0; nTail < nPairs; nTail++) {
          merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
        }

        bLen = tailOffset;
      }

      if (n > bLen) {
        merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
      }
    }

    //
    // Arguments    : array<int, 1U> &idx
    //                array<unsigned int, 1U> &x
    //                int offset
    //                int n
    //                int preSortLevel
    //                array<int, 1U> &iwork
    //                array<unsigned int, 1U> &xwork
    // Return Type  : void
    //
    static void merge_block(array<int, 1U> &idx, array<unsigned int, 1U> &x, int
      offset, int n, int preSortLevel, array<int, 1U> &iwork, array<unsigned int,
      1U> &xwork)
    {
      int bLen;
      int nPairs;
      nPairs = n >> preSortLevel;
      bLen = 1 << preSortLevel;
      while (nPairs > 1) {
        int nTail;
        int tailOffset;
        if ((nPairs & 1) != 0) {
          nPairs--;
          tailOffset = bLen * nPairs;
          nTail = n - tailOffset;
          if (nTail > bLen) {
            merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
          }
        }

        tailOffset = bLen << 1;
        nPairs >>= 1;
        for (nTail = 0; nTail < nPairs; nTail++) {
          merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
        }

        bLen = tailOffset;
      }

      if (n > bLen) {
        merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
      }
    }

    //
    // Arguments    : double A[6]
    //                const double B[36]
    // Return Type  : void
    //
    static void mrdiv(double A[6], const double B[36])
    {
      double b_A[36];
      double temp;
      int ipiv[6];
      int i;
      int jAcol;
      std::copy(&B[0], &B[36], &b_A[0]);
      reflapack::xzgetrf(b_A, ipiv);
      for (int j{0}; j < 6; j++) {
        jAcol = 6 * j;
        for (int k{0}; k < j; k++) {
          temp = b_A[k + jAcol];
          if (temp != 0.0) {
            A[j] -= temp * A[k];
          }
        }

        A[j] *= 1.0 / b_A[j + jAcol];
      }

      for (int j{5}; j >= 0; j--) {
        jAcol = 6 * j - 1;
        i = j + 2;
        for (int k{i}; k < 7; k++) {
          temp = b_A[k + jAcol];
          if (temp != 0.0) {
            A[j] -= temp * A[k - 1];
          }
        }
      }

      for (int j{4}; j >= 0; j--) {
        i = ipiv[j];
        if (i != j + 1) {
          temp = A[j];
          A[j] = A[i - 1];
          A[i - 1] = temp;
        }
      }
    }

    //
    // Arguments    : const array<double, 2U> &A
    //                const array<double, 1U> &B
    //                array<double, 1U> &Y
    // Return Type  : void
    //
    static void qrsolve(const array<double, 2U> &A, const array<double, 1U> &B,
                        array<double, 1U> &Y)
    {
      array<double, 2U> b_A;
      array<double, 1U> tau;
      array<double, 1U> vn1;
      array<double, 1U> vn2;
      array<double, 1U> work;
      array<int, 2U> jpvt;
      double smax;
      int b_i;
      int coltop;
      int i;
      int ix;
      int k;
      int m;
      int ma;
      int n;
      int pvt;
      int u1;
      b_A.set_size(A.size(0), A.size(1));
      i = A.size(0) * A.size(1);
      for (b_i = 0; b_i < i; b_i++) {
        b_A[b_i] = A[b_i];
      }

      m = A.size(0);
      n = A.size(1);
      i = A.size(0);
      u1 = A.size(1);
      if (i <= u1) {
        u1 = i;
      }

      tau.set_size(u1);
      for (b_i = 0; b_i < u1; b_i++) {
        tau[b_i] = 0.0;
      }

      jpvt.set_size(1, A.size(1));
      i = A.size(1);
      ma = A.size(0);
      work.set_size(A.size(1));
      vn1.set_size(A.size(1));
      vn2.set_size(A.size(1));
      for (k = 0; k < i; k++) {
        jpvt[k] = k + 1;
        work[k] = 0.0;
        smax = blas::xnrm2(m, A, k * ma + 1);
        vn1[k] = smax;
        vn2[k] = smax;
      }

      for (int c_i{0}; c_i < u1; c_i++) {
        double s;
        double temp2;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = c_i + 2;
        coltop = c_i * ma;
        ii = coltop + c_i;
        nmi = n - c_i;
        mmi = m - c_i;
        if (nmi < 1) {
          i = -1;
        } else {
          i = 0;
          if (nmi > 1) {
            smax = std::abs(vn1[c_i]);
            for (k = 2; k <= nmi; k++) {
              s = std::abs(vn1[(c_i + k) - 1]);
              if (s > smax) {
                i = k - 1;
                smax = s;
              }
            }
          }
        }

        pvt = c_i + i;
        if (pvt + 1 != c_i + 1) {
          ix = pvt * ma;
          for (k = 0; k < m; k++) {
            i = ix + k;
            smax = b_A[i];
            b_i = coltop + k;
            b_A[i] = b_A[b_i];
            b_A[b_i] = smax;
          }

          ix = jpvt[pvt];
          jpvt[pvt] = jpvt[c_i];
          jpvt[c_i] = ix;
          vn1[pvt] = vn1[c_i];
          vn2[pvt] = vn2[c_i];
        }

        if (c_i + 1 < m) {
          temp2 = b_A[ii];
          coltop = ii + 2;
          tau[c_i] = 0.0;
          if (mmi > 0) {
            smax = blas::xnrm2(mmi - 1, b_A, ii + 2);
            if (smax != 0.0) {
              s = rt_hypotd_snf(b_A[ii], smax);
              if (b_A[ii] >= 0.0) {
                s = -s;
              }

              if (std::abs(s) < 1.0020841800044864E-292) {
                ix = 0;
                b_i = ii + mmi;
                do {
                  ix++;
                  for (k = coltop; k <= b_i; k++) {
                    b_A[k - 1] = 9.9792015476736E+291 * b_A[k - 1];
                  }

                  s *= 9.9792015476736E+291;
                  temp2 *= 9.9792015476736E+291;
                } while ((std::abs(s) < 1.0020841800044864E-292) && (ix < 20));

                s = rt_hypotd_snf(temp2, blas::xnrm2(mmi - 1, b_A, ii + 2));
                if (temp2 >= 0.0) {
                  s = -s;
                }

                tau[c_i] = (s - temp2) / s;
                smax = 1.0 / (temp2 - s);
                for (k = coltop; k <= b_i; k++) {
                  b_A[k - 1] = smax * b_A[k - 1];
                }

                for (k = 0; k < ix; k++) {
                  s *= 1.0020841800044864E-292;
                }

                temp2 = s;
              } else {
                tau[c_i] = (s - b_A[ii]) / s;
                smax = 1.0 / (b_A[ii] - s);
                b_i = ii + mmi;
                for (k = coltop; k <= b_i; k++) {
                  b_A[k - 1] = smax * b_A[k - 1];
                }

                temp2 = s;
              }
            }
          }

          b_A[ii] = temp2;
        } else {
          tau[c_i] = 0.0;
        }

        if (c_i + 1 < n) {
          int jA;
          int lastv;
          temp2 = b_A[ii];
          b_A[ii] = 1.0;
          jA = (ii + ma) + 1;
          if (tau[c_i] != 0.0) {
            boolean_T exitg2;
            lastv = mmi - 1;
            i = (ii + mmi) - 1;
            while ((lastv + 1 > 0) && (b_A[i] == 0.0)) {
              lastv--;
              i--;
            }

            ix = nmi - 2;
            exitg2 = false;
            while ((!exitg2) && (ix + 1 > 0)) {
              int exitg1;
              coltop = jA + ix * ma;
              k = coltop;
              do {
                exitg1 = 0;
                if (k <= coltop + lastv) {
                  if (b_A[k - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    k++;
                  }
                } else {
                  ix--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }
          } else {
            lastv = -1;
            ix = -1;
          }

          if (lastv + 1 > 0) {
            if (ix + 1 != 0) {
              for (coltop = 0; coltop <= ix; coltop++) {
                work[coltop] = 0.0;
              }

              coltop = 0;
              b_i = jA + ma * ix;
              for (pvt = jA; ma < 0 ? pvt >= b_i : pvt <= b_i; pvt += ma) {
                smax = 0.0;
                i = pvt + lastv;
                for (k = pvt; k <= i; k++) {
                  smax += b_A[k - 1] * b_A[(ii + k) - pvt];
                }

                work[coltop] = work[coltop] + smax;
                coltop++;
              }
            }

            if (!(-tau[c_i] == 0.0)) {
              for (pvt = 0; pvt <= ix; pvt++) {
                if (work[pvt] != 0.0) {
                  smax = work[pvt] * -tau[c_i];
                  b_i = lastv + jA;
                  for (i = jA; i <= b_i; i++) {
                    b_A[i - 1] = b_A[i - 1] + b_A[(ii + i) - jA] * smax;
                  }
                }

                jA += ma;
              }
            }
          }

          b_A[ii] = temp2;
        }

        for (pvt = ip1; pvt <= n; pvt++) {
          i = c_i + (pvt - 1) * ma;
          smax = vn1[pvt - 1];
          if (smax != 0.0) {
            s = std::abs(b_A[i]) / smax;
            s = 1.0 - s * s;
            if (s < 0.0) {
              s = 0.0;
            }

            temp2 = smax / vn2[pvt - 1];
            temp2 = s * (temp2 * temp2);
            if (temp2 <= 1.4901161193847656E-8) {
              if (c_i + 1 < m) {
                smax = blas::xnrm2(mmi - 1, b_A, i + 2);
                vn1[pvt - 1] = smax;
                vn2[pvt - 1] = smax;
              } else {
                vn1[pvt - 1] = 0.0;
                vn2[pvt - 1] = 0.0;
              }
            } else {
              vn1[pvt - 1] = smax * std::sqrt(s);
            }
          }
        }
      }

      coltop = 0;
      if (b_A.size(0) < b_A.size(1)) {
        i = b_A.size(0);
        ix = b_A.size(1);
      } else {
        i = b_A.size(1);
        ix = b_A.size(0);
      }

      smax = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 *
                       static_cast<double>(ix)) * std::abs(b_A[0]);
      while ((coltop < i) && (!(std::abs(b_A[coltop + b_A.size(0) * coltop]) <=
               smax))) {
        coltop++;
      }

      work.set_size(B.size(0));
      i = B.size(0);
      for (b_i = 0; b_i < i; b_i++) {
        work[b_i] = B[b_i];
      }

      Y.set_size(b_A.size(1));
      i = b_A.size(1);
      for (b_i = 0; b_i < i; b_i++) {
        Y[b_i] = 0.0;
      }

      m = b_A.size(0);
      for (pvt = 0; pvt < u1; pvt++) {
        if (tau[pvt] != 0.0) {
          smax = work[pvt];
          b_i = pvt + 2;
          for (int c_i{b_i}; c_i <= m; c_i++) {
            smax += b_A[(c_i + b_A.size(0) * pvt) - 1] * work[c_i - 1];
          }

          smax *= tau[pvt];
          if (smax != 0.0) {
            work[pvt] = work[pvt] - smax;
            for (int c_i{b_i}; c_i <= m; c_i++) {
              work[c_i - 1] = work[c_i - 1] - b_A[(c_i + b_A.size(0) * pvt) - 1]
                * smax;
            }
          }
        }
      }

      for (int c_i{0}; c_i < coltop; c_i++) {
        Y[jpvt[c_i] - 1] = work[c_i];
      }

      for (pvt = coltop; pvt >= 1; pvt--) {
        b_i = jpvt[pvt - 1];
        Y[b_i - 1] = Y[b_i - 1] / b_A[(pvt + b_A.size(0) * (pvt - 1)) - 1];
        for (int c_i{0}; c_i <= pvt - 2; c_i++) {
          Y[jpvt[c_i] - 1] = Y[jpvt[c_i] - 1] - Y[b_i - 1] * b_A[c_i + b_A.size
            (0) * (pvt - 1)];
        }
      }
    }

    //
    // Arguments    : int n
    //                double z[36]
    //                int iz0
    //                const double cs[10]
    //                int ic0
    //                int is0
    // Return Type  : void
    //
    namespace reflapack
    {
      static void b_rotateRight(int n, double z[36], int iz0, const double cs[10],
        int ic0, int is0)
      {
        for (int j{0}; j <= n - 2; j++) {
          double ctemp;
          double stemp;
          int offsetj;
          int offsetjp1;
          ctemp = cs[(ic0 + j) - 1];
          stemp = cs[(is0 + j) - 1];
          offsetj = (j * 6 + iz0) - 2;
          offsetjp1 = ((j + 1) * 6 + iz0) - 2;
          if ((ctemp != 1.0) || (stemp != 0.0)) {
            for (int i{0}; i < 6; i++) {
              double temp;
              int b_i;
              int temp_tmp;
              temp_tmp = (offsetjp1 + i) + 1;
              temp = z[temp_tmp];
              b_i = (offsetj + i) + 1;
              z[temp_tmp] = ctemp * temp - stemp * z[b_i];
              z[b_i] = stemp * temp + ctemp * z[b_i];
            }
          }
        }
      }

      //
      // Arguments    : double cfrom
      //                double cto
      //                int m
      //                double A[5]
      //                int iA0
      // Return Type  : void
      //
      static void b_xzlascl(double cfrom, double cto, int m, double A[5], int
                            iA0)
      {
        double cfromc;
        double ctoc;
        boolean_T notdone;
        cfromc = cfrom;
        ctoc = cto;
        notdone = true;
        while (notdone) {
          double cfrom1;
          double cto1;
          double mul;
          cfrom1 = cfromc * 2.0041683600089728E-292;
          cto1 = ctoc / 4.9896007738368E+291;
          if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
            mul = 2.0041683600089728E-292;
            cfromc = cfrom1;
          } else if (std::abs(cto1) > std::abs(cfromc)) {
            mul = 4.9896007738368E+291;
            ctoc = cto1;
          } else {
            mul = ctoc / cfromc;
            notdone = false;
          }

          for (int i{0}; i < m; i++) {
            int b_i;
            b_i = (iA0 + i) - 1;
            A[b_i] *= mul;
          }
        }
      }

      //
      // Arguments    : int n
      //                double z[36]
      //                int iz0
      //                const double cs[10]
      //                int ic0
      //                int is0
      // Return Type  : void
      //
      static void rotateRight(int n, double z[36], int iz0, const double cs[10],
        int ic0, int is0)
      {
        int i;
        i = n - 1;
        for (int j{i}; j >= 1; j--) {
          double ctemp;
          double stemp;
          int offsetj;
          int offsetjp1;
          ctemp = cs[(ic0 + j) - 2];
          stemp = cs[(is0 + j) - 2];
          offsetj = ((j - 1) * 6 + iz0) - 2;
          offsetjp1 = (j * 6 + iz0) - 2;
          if ((ctemp != 1.0) || (stemp != 0.0)) {
            for (int b_i{0}; b_i < 6; b_i++) {
              double temp;
              int i1;
              int temp_tmp;
              temp_tmp = (offsetjp1 + b_i) + 1;
              temp = z[temp_tmp];
              i1 = (offsetj + b_i) + 1;
              z[temp_tmp] = ctemp * temp - stemp * z[i1];
              z[i1] = stemp * temp + ctemp * z[i1];
            }
          }
        }
      }

      //
      // Arguments    : double a
      //                double b
      //                double c
      //                double d
      //                double &q
      // Return Type  : double
      //
      static double xdladiv(double a, double b, double c, double d, double &q)
      {
        double aa;
        double ab;
        double bb;
        double cc;
        double cd;
        double cd_tmp;
        double dd;
        double p;
        double r;
        double s;
        aa = a;
        bb = b;
        cc = c;
        dd = d;
        ab = std::fmax(std::abs(a), std::abs(b));
        cd_tmp = std::abs(d);
        r = std::abs(c);
        cd = std::fmax(r, cd_tmp);
        s = 1.0;
        if (ab >= 8.9884656743115785E+307) {
          aa = 0.5 * a;
          bb = 0.5 * b;
          s = 2.0;
        }

        if (cd >= 8.9884656743115785E+307) {
          cc = 0.5 * c;
          dd = 0.5 * d;
          s *= 0.5;
        }

        if (ab <= 2.0041683600089728E-292) {
          aa *= 4.0564819207303341E+31;
          bb *= 4.0564819207303341E+31;
          s /= 4.0564819207303341E+31;
        }

        if (cd <= 2.0041683600089728E-292) {
          cc *= 4.0564819207303341E+31;
          dd *= 4.0564819207303341E+31;
          s *= 4.0564819207303341E+31;
        }

        if (cd_tmp <= r) {
          r = dd / cc;
          cd = 1.0 / (cc + dd * r);
          if (r != 0.0) {
            ab = bb * r;
            cd_tmp = bb * cd;
            if (ab != 0.0) {
              p = (aa + ab) * cd;
            } else {
              p = aa * cd + cd_tmp * r;
            }

            ab = -aa * r;
            if (ab != 0.0) {
              q = (bb + ab) * cd;
            } else {
              q = cd_tmp + -aa * cd * r;
            }
          } else {
            p = (aa + dd * (bb / cc)) * cd;
            q = (bb + dd * (-aa / cc)) * cd;
          }
        } else {
          r = cc / dd;
          cd = 1.0 / (dd + cc * r);
          if (r != 0.0) {
            ab = aa * r;
            cd_tmp = aa * cd;
            if (ab != 0.0) {
              p = (bb + ab) * cd;
            } else {
              p = bb * cd + cd_tmp * r;
            }

            ab = -bb * r;
            if (ab != 0.0) {
              q = (aa + ab) * cd;
            } else {
              q = cd_tmp + -bb * cd * r;
            }
          } else {
            p = (bb + cc * (aa / dd)) * cd;
            q = (aa + cc * (-bb / dd)) * cd;
          }

          q = -q;
        }

        p *= s;
        q *= s;
        return p;
      }

      //
      // Arguments    : double a
      //                double b
      //                double c
      //                double &rt2
      //                double &cs1
      //                double &sn1
      // Return Type  : double
      //
      static double xdlaev2(double a, double b, double c, double &rt2, double
                            &cs1, double &sn1)
      {
        double ab;
        double acmn;
        double acmx;
        double adf;
        double df;
        double rt1;
        double sm;
        double tb;
        int sgn1;
        int sgn2;
        sm = a + c;
        df = a - c;
        adf = std::abs(df);
        tb = b + b;
        ab = std::abs(tb);
        if (std::abs(a) > std::abs(c)) {
          acmx = a;
          acmn = c;
        } else {
          acmx = c;
          acmn = a;
        }

        if (adf > ab) {
          double b_a;
          b_a = ab / adf;
          adf *= std::sqrt(b_a * b_a + 1.0);
        } else if (adf < ab) {
          double b_a;
          b_a = adf / ab;
          adf = ab * std::sqrt(b_a * b_a + 1.0);
        } else {
          adf = ab * 1.4142135623730951;
        }

        if (sm < 0.0) {
          rt1 = 0.5 * (sm - adf);
          sgn1 = -1;
          rt2 = acmx / rt1 * acmn - b / rt1 * b;
        } else if (sm > 0.0) {
          rt1 = 0.5 * (sm + adf);
          sgn1 = 1;
          rt2 = acmx / rt1 * acmn - b / rt1 * b;
        } else {
          rt1 = 0.5 * adf;
          rt2 = -0.5 * adf;
          sgn1 = 1;
        }

        if (df >= 0.0) {
          adf += df;
          sgn2 = 1;
        } else {
          adf = df - adf;
          sgn2 = -1;
        }

        if (std::abs(adf) > ab) {
          adf = -tb / adf;
          sn1 = 1.0 / std::sqrt(adf * adf + 1.0);
          cs1 = adf * sn1;
        } else if (ab == 0.0) {
          cs1 = 1.0;
          sn1 = 0.0;
        } else {
          adf = -adf / tb;
          cs1 = 1.0 / std::sqrt(adf * adf + 1.0);
          sn1 = adf * cs1;
        }

        if (sgn1 == sgn2) {
          adf = cs1;
          cs1 = -sn1;
          sn1 = adf;
        }

        return rt1;
      }

      //
      // Arguments    : int ilo
      //                int ihi
      //                double h[36]
      //                int iloz
      //                int ihiz
      //                double z[36]
      //                double wr[6]
      //                double wi[6]
      // Return Type  : int
      //
      static int xdlahqr(int ilo, int ihi, double h[36], int iloz, int ihiz,
                         double z[36], double wr[6], double wi[6])
      {
        double aa;
        double d;
        double h22;
        double rt1r;
        double rt2r;
        double s;
        double s_tmp_tmp;
        double tst;
        int b_i;
        int i;
        int info;
        info = 0;
        i = static_cast<unsigned char>(ilo - 1);
        for (b_i = 0; b_i < i; b_i++) {
          wr[b_i] = h[b_i + 6 * b_i];
          wi[b_i] = 0.0;
        }

        i = ihi + 1;
        for (b_i = i; b_i < 7; b_i++) {
          wr[b_i - 1] = h[(b_i + 6 * (b_i - 1)) - 1];
          wi[b_i - 1] = 0.0;
        }

        if (ilo == ihi) {
          wr[ilo - 1] = h[(ilo + 6 * (ilo - 1)) - 1];
          wi[ilo - 1] = 0.0;
        } else {
          double smlnum;
          int i1;
          int kdefl;
          int nr;
          int nz;
          boolean_T exitg1;
          i = ihi - 3;
          for (nr = ilo; nr <= i; nr++) {
            i1 = nr + 6 * (nr - 1);
            h[i1 + 1] = 0.0;
            h[i1 + 2] = 0.0;
          }

          if (ilo <= ihi - 2) {
            h[(ihi + 6 * (ihi - 3)) - 1] = 0.0;
          }

          nz = (ihiz - iloz) + 1;
          smlnum = 2.2250738585072014E-308 * (static_cast<double>((ihi - ilo) +
            1) / 2.2204460492503131E-16);
          kdefl = 0;
          b_i = ihi - 1;
          exitg1 = false;
          while ((!exitg1) && (b_i + 1 >= ilo)) {
            double h21;
            int b_k;
            int its;
            int ix;
            int k;
            int l;
            int temp_tmp_tmp;
            boolean_T converged;
            boolean_T exitg2;
            l = ilo;
            converged = false;
            its = 0;
            exitg2 = false;
            while ((!exitg2) && (its < 301)) {
              double tr;
              boolean_T exitg3;
              k = b_i;
              exitg3 = false;
              while ((!exitg3) && (k + 1 > l)) {
                i = k + 6 * (k - 1);
                d = std::abs(h[i]);
                if (d <= smlnum) {
                  exitg3 = true;
                } else {
                  ix = k + 6 * k;
                  h21 = h[ix];
                  tr = std::abs(h21);
                  aa = h[i - 1];
                  tst = std::abs(aa) + tr;
                  if (tst == 0.0) {
                    if (k - 1 >= ilo) {
                      tst = std::abs(h[(k + 6 * (k - 2)) - 1]);
                    }

                    if (k + 2 <= ihi) {
                      tst += std::abs(h[ix + 1]);
                    }
                  }

                  if (d <= 2.2204460492503131E-16 * tst) {
                    h22 = std::abs(h[ix - 1]);
                    h21 = std::abs(aa - h21);
                    aa = std::fmax(tr, h21);
                    tst = std::fmin(tr, h21);
                    s = aa + tst;
                    if (std::fmin(d, h22) * (std::fmax(d, h22) / s) <= std::fmax
                        (smlnum, 2.2204460492503131E-16 * (tst * (aa / s)))) {
                      exitg3 = true;
                    } else {
                      k--;
                    }
                  } else {
                    k--;
                  }
                }
              }

              l = k + 1;
              if (k + 1 > ilo) {
                h[k + 6 * (k - 1)] = 0.0;
              }

              if (k + 1 >= b_i) {
                converged = true;
                exitg2 = true;
              } else {
                double v[3];
                int m;
                kdefl++;
                if (kdefl - kdefl / 20 * 20 == 0) {
                  s = std::abs(h[b_i + 6 * (b_i - 1)]) + std::abs(h[(b_i + 6 *
                    (b_i - 2)) - 1]);
                  tst = 0.75 * s + h[b_i + 6 * b_i];
                  aa = -0.4375 * s;
                  h21 = s;
                  h22 = tst;
                } else if (kdefl - kdefl / 10 * 10 == 0) {
                  ix = k + 6 * k;
                  s = std::abs(h[ix + 1]) + std::abs(h[(k + 6 * (k + 1)) + 2]);
                  tst = 0.75 * s + h[ix];
                  aa = -0.4375 * s;
                  h21 = s;
                  h22 = tst;
                } else {
                  ix = b_i + 6 * (b_i - 1);
                  tst = h[ix - 1];
                  h21 = h[ix];
                  ix = b_i + 6 * b_i;
                  aa = h[ix - 1];
                  h22 = h[ix];
                }

                s = ((std::abs(tst) + std::abs(aa)) + std::abs(h21)) + std::abs
                  (h22);
                if (s == 0.0) {
                  rt1r = 0.0;
                  tr = 0.0;
                  rt2r = 0.0;
                  h22 = 0.0;
                } else {
                  tst /= s;
                  h21 /= s;
                  aa /= s;
                  h22 /= s;
                  tr = (tst + h22) / 2.0;
                  tst = (tst - tr) * (h22 - tr) - aa * h21;
                  h21 = std::sqrt(std::abs(tst));
                  if (tst >= 0.0) {
                    rt1r = tr * s;
                    rt2r = rt1r;
                    tr = h21 * s;
                    h22 = -tr;
                  } else {
                    rt1r = tr + h21;
                    rt2r = tr - h21;
                    if (std::abs(rt1r - h22) <= std::abs(rt2r - h22)) {
                      rt1r *= s;
                      rt2r = rt1r;
                    } else {
                      rt2r *= s;
                      rt1r = rt2r;
                    }

                    tr = 0.0;
                    h22 = 0.0;
                  }
                }

                m = b_i - 1;
                exitg3 = false;
                while ((!exitg3) && (m >= k + 1)) {
                  ix = m + 6 * (m - 1);
                  tst = h[ix];
                  s_tmp_tmp = h[ix - 1];
                  h21 = s_tmp_tmp - rt2r;
                  s = (std::abs(h21) + std::abs(h22)) + std::abs(tst);
                  aa = tst / s;
                  ix = m + 6 * m;
                  v[0] = (aa * h[ix - 1] + h21 * (h21 / s)) - tr * (h22 / s);
                  tst = h[ix];
                  v[1] = aa * (((s_tmp_tmp + tst) - rt1r) - rt2r);
                  v[2] = aa * h[ix + 1];
                  s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
                  v[0] /= s;
                  v[1] /= s;
                  v[2] /= s;
                  if (m == k + 1) {
                    exitg3 = true;
                  } else {
                    i = m + 6 * (m - 2);
                    if (std::abs(h[i - 1]) * (std::abs(v[1]) + std::abs(v[2])) <=
                        2.2204460492503131E-16 * std::abs(v[0]) * ((std::abs(h[i
                           - 2]) + std::abs(s_tmp_tmp)) + std::abs(tst))) {
                      exitg3 = true;
                    } else {
                      m--;
                    }
                  }
                }

                for (int c_k{m}; c_k <= b_i; c_k++) {
                  ix = (b_i - c_k) + 2;
                  if (ix >= 3) {
                    nr = 3;
                  } else {
                    nr = ix;
                  }

                  if (c_k > m) {
                    ix = ((c_k - 2) * 6 + c_k) - 1;
                    for (b_k = 0; b_k < nr; b_k++) {
                      v[b_k] = h[ix + b_k];
                    }
                  }

                  tst = v[0];
                  tr = xzlarfg(nr, tst, v);
                  if (c_k > m) {
                    i = c_k + 6 * (c_k - 2);
                    h[i - 1] = tst;
                    h[i] = 0.0;
                    if (c_k < b_i) {
                      h[i + 1] = 0.0;
                    }
                  } else if (m > k + 1) {
                    i = (c_k + 6 * (c_k - 2)) - 1;
                    h[i] *= 1.0 - tr;
                  }

                  d = v[1];
                  tst = tr * v[1];
                  if (nr == 3) {
                    s_tmp_tmp = v[2];
                    aa = tr * v[2];
                    for (nr = c_k; nr < 7; nr++) {
                      i = c_k + 6 * (nr - 1);
                      rt2r = h[i - 1];
                      rt1r = h[i];
                      s = h[i + 1];
                      h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                      rt2r -= h21 * tr;
                      h[i - 1] = rt2r;
                      rt1r -= h21 * tst;
                      h[i] = rt1r;
                      s -= h21 * aa;
                      h[i + 1] = s;
                    }

                    if (c_k + 3 <= b_i + 1) {
                      i = c_k;
                    } else {
                      i = b_i - 2;
                    }

                    i = static_cast<unsigned char>(i + 3);
                    for (nr = 0; nr < i; nr++) {
                      i1 = nr + 6 * (c_k - 1);
                      rt2r = h[i1];
                      ix = nr + 6 * c_k;
                      rt1r = h[ix];
                      temp_tmp_tmp = nr + 6 * (c_k + 1);
                      s = h[temp_tmp_tmp];
                      h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                      rt2r -= h21 * tr;
                      h[i1] = rt2r;
                      rt1r -= h21 * tst;
                      h[ix] = rt1r;
                      s -= h21 * aa;
                      h[temp_tmp_tmp] = s;
                    }

                    for (nr = iloz; nr <= ihiz; nr++) {
                      i = (nr + 6 * (c_k - 1)) - 1;
                      rt2r = z[i];
                      i1 = (nr + 6 * c_k) - 1;
                      rt1r = z[i1];
                      ix = (nr + 6 * (c_k + 1)) - 1;
                      s = z[ix];
                      h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                      rt2r -= h21 * tr;
                      z[i] = rt2r;
                      rt1r -= h21 * tst;
                      z[i1] = rt1r;
                      s -= h21 * aa;
                      z[ix] = s;
                    }
                  } else if (nr == 2) {
                    for (nr = c_k; nr < 7; nr++) {
                      i = c_k + 6 * (nr - 1);
                      s_tmp_tmp = h[i - 1];
                      rt2r = h[i];
                      h21 = s_tmp_tmp + d * rt2r;
                      s_tmp_tmp -= h21 * tr;
                      h[i - 1] = s_tmp_tmp;
                      rt2r -= h21 * tst;
                      h[i] = rt2r;
                    }

                    i = static_cast<unsigned char>(b_i + 1);
                    for (nr = 0; nr < i; nr++) {
                      i1 = nr + 6 * (c_k - 1);
                      s_tmp_tmp = h[i1];
                      ix = nr + 6 * c_k;
                      rt2r = h[ix];
                      h21 = s_tmp_tmp + d * rt2r;
                      s_tmp_tmp -= h21 * tr;
                      h[i1] = s_tmp_tmp;
                      rt2r -= h21 * tst;
                      h[ix] = rt2r;
                    }

                    for (nr = iloz; nr <= ihiz; nr++) {
                      i = (nr + 6 * (c_k - 1)) - 1;
                      s_tmp_tmp = z[i];
                      i1 = (nr + 6 * c_k) - 1;
                      rt2r = z[i1];
                      h21 = s_tmp_tmp + d * rt2r;
                      s_tmp_tmp -= h21 * tr;
                      z[i] = s_tmp_tmp;
                      rt2r -= h21 * tst;
                      z[i1] = rt2r;
                    }
                  }
                }

                its++;
              }
            }

            if (!converged) {
              info = b_i + 1;
              exitg1 = true;
            } else {
              if (l == b_i + 1) {
                wr[b_i] = h[b_i + 6 * b_i];
                wi[b_i] = 0.0;
              } else if (l == b_i) {
                i = b_i + 6 * b_i;
                d = h[i - 1];
                i1 = 6 * (b_i - 1);
                ix = b_i + i1;
                s_tmp_tmp = h[ix];
                rt2r = h[i];
                wr[b_i - 1] = xdlanv2(h[ix - 1], d, s_tmp_tmp, rt2r, wi[b_i - 1],
                                      rt1r, s, aa, h22);
                wr[b_i] = rt1r;
                wi[b_i] = s;
                h[i - 1] = d;
                h[ix] = s_tmp_tmp;
                h[i] = rt2r;
                if (b_i + 1 < 6) {
                  ix = (b_i + 1) * 6 + b_i;
                  i = static_cast<unsigned char>(5 - b_i);
                  for (k = 0; k < i; k++) {
                    nr = ix + k * 6;
                    tst = h[nr];
                    h21 = h[nr - 1];
                    h[nr] = aa * tst - h22 * h21;
                    h[nr - 1] = aa * h21 + h22 * tst;
                  }
                }

                if (b_i - 1 >= 1) {
                  nr = b_i * 6;
                  i = static_cast<unsigned char>(b_i - 1);
                  for (k = 0; k < i; k++) {
                    b_k = nr + k;
                    tst = h[b_k];
                    temp_tmp_tmp = i1 + k;
                    h21 = h[temp_tmp_tmp];
                    h[b_k] = aa * tst - h22 * h21;
                    h[temp_tmp_tmp] = aa * h21 + h22 * tst;
                  }
                }

                if (nz >= 1) {
                  ix = (i1 + iloz) - 1;
                  nr = (b_i * 6 + iloz) - 1;
                  i = static_cast<unsigned char>(nz);
                  for (k = 0; k < i; k++) {
                    b_k = nr + k;
                    tst = z[b_k];
                    temp_tmp_tmp = ix + k;
                    h21 = z[temp_tmp_tmp];
                    z[b_k] = aa * tst - h22 * h21;
                    z[temp_tmp_tmp] = aa * h21 + h22 * tst;
                  }
                }
              }

              kdefl = 0;
              b_i = l - 2;
            }
          }

          for (nr = 0; nr < 4; nr++) {
            for (b_i = nr + 3; b_i < 7; b_i++) {
              h[(b_i + 6 * nr) - 1] = 0.0;
            }
          }
        }

        return info;
      }

      //
      // Arguments    : int na
      //                int nw
      //                double smin
      //                const double A[36]
      //                int ia0
      //                const double B[18]
      //                int ib0
      //                double wr
      //                double wi
      //                double X[4]
      //                double &xnorm
      // Return Type  : double
      //
      static double xdlaln2(int na, int nw, double smin, const double A[36], int
                            ia0, const double B[18], int ib0, double wr, double
                            wi, double X[4], double &xnorm)
      {
        static const signed char ipivot[16]{ 1, 2, 3, 4, 2, 1, 4, 3, 3, 4, 1, 2,
          4, 3, 2, 1 };

        double scale;
        scale = 1.0;
        if (na == 1) {
          if (nw == 1) {
            double cr21;
            double cr22;
            double lr21;
            double ui12;
            cr21 = A[ia0 - 1] - wr;
            ui12 = std::abs(cr21);
            if (ui12 < smin) {
              cr21 = smin;
              ui12 = smin;
            }

            cr22 = B[ib0 - 1];
            lr21 = std::abs(cr22);
            if ((ui12 < 1.0) && (lr21 > 1.0) && (lr21 > 2.2471164185778949E+307 *
                 ui12)) {
              scale = 1.0 / lr21;
            }

            X[0] = cr22 * scale / cr21;
            xnorm = std::abs(X[0]);
          } else {
            double cr21;
            double cr22;
            double lr21;
            double temp;
            double ui12;
            double ur12;
            cr21 = A[ia0 - 1] - wr;
            temp = -wi;
            ui12 = std::abs(cr21) + std::abs(-wi);
            if (ui12 < smin) {
              cr21 = smin;
              temp = 0.0;
              ui12 = smin;
            }

            cr22 = B[ib0 - 1];
            ur12 = B[ib0 + 5];
            lr21 = std::abs(cr22) + std::abs(ur12);
            if ((ui12 < 1.0) && (lr21 > 1.0) && (lr21 > 2.2471164185778949E+307 *
                 ui12)) {
              scale = 1.0 / lr21;
            }

            X[0] = xdladiv(scale * cr22, scale * ur12, cr21, temp, X[2]);
            xnorm = std::abs(X[0]) + std::abs(X[2]);
          }
        } else {
          double cr[4];
          cr[0] = A[ia0 - 1] - wr;
          cr[3] = A[ia0 + 6] - wr;
          cr[1] = A[ia0];
          cr[2] = A[ia0 + 5];
          if (nw == 1) {
            double cmax;
            double cr21;
            int icmax;
            cmax = 0.0;
            icmax = -1;
            cr21 = std::abs(cr[0]);
            if (cr21 > 0.0) {
              cmax = cr21;
              icmax = 0;
            }

            cr21 = std::abs(cr[1]);
            if (cr21 > cmax) {
              cmax = cr21;
              icmax = 1;
            }

            cr21 = std::abs(cr[2]);
            if (cr21 > cmax) {
              cmax = cr21;
              icmax = 2;
            }

            cr21 = std::abs(cr[3]);
            if (cr21 > cmax) {
              cmax = cr21;
              icmax = 3;
            }

            if (cmax < smin) {
              double cr22;
              double lr21;
              double temp;
              cr22 = B[ib0 - 1];
              lr21 = std::fmax(std::abs(cr22), std::abs(B[ib0]));
              if ((smin < 1.0) && (lr21 > 1.0) && (lr21 >
                   2.2471164185778949E+307 * smin)) {
                scale = 1.0 / lr21;
              }

              temp = scale / smin;
              X[0] = temp * cr22;
              X[1] = temp * B[ib0];
              xnorm = temp * lr21;
            } else {
              double br1;
              double br2;
              double cr22;
              double lr21;
              double temp;
              double ur11r;
              double ur12;
              int ur12_tmp;
              ur12_tmp = icmax << 2;
              ur12 = cr[ipivot[ur12_tmp + 2] - 1];
              ur11r = 1.0 / cr[icmax];
              lr21 = ur11r * cr[ipivot[ur12_tmp + 1] - 1];
              cr22 = cr[ipivot[ur12_tmp + 3] - 1] - ur12 * lr21;
              if (std::abs(cr22) < smin) {
                cr22 = smin;
              }

              if ((icmax + 1 == 2) || (icmax + 1 == 4)) {
                br1 = B[ib0];
                br2 = B[ib0 - 1];
              } else {
                br1 = B[ib0 - 1];
                br2 = B[ib0];
              }

              br2 -= lr21 * br1;
              lr21 = std::fmax(std::abs(br1 * (cr22 * ur11r)), std::abs(br2));
              if (lr21 > 1.0) {
                cr21 = std::abs(cr22);
                if ((cr21 < 1.0) && (lr21 >= 2.2471164185778949E+307 * cr21)) {
                  scale = 1.0 / lr21;
                }
              }

              cr22 = br2 * scale / cr22;
              temp = scale * br1 * ur11r - cr22 * (ur11r * ur12);
              if ((icmax + 1 == 3) || (icmax + 1 == 4)) {
                X[0] = cr22;
                X[1] = temp;
              } else {
                X[0] = temp;
                X[1] = cr22;
              }

              xnorm = std::fmax(std::abs(temp), std::abs(cr22));
              if ((xnorm > 1.0) && (cmax > 1.0) && (xnorm >
                   2.2471164185778949E+307 / cmax)) {
                temp = cmax / 2.2471164185778949E+307;
                X[0] *= temp;
                X[1] *= temp;
                xnorm *= temp;
                scale *= temp;
              }
            }
          } else {
            double ci[4];
            double cmax;
            double cr21;
            double temp;
            int icmax;
            ci[0] = -wi;
            ci[1] = 0.0;
            ci[2] = 0.0;
            ci[3] = -wi;
            cmax = 0.0;
            icmax = -1;
            cr21 = std::abs(-wi);
            temp = std::abs(cr[0]) + cr21;
            if (temp > 0.0) {
              cmax = temp;
              icmax = 0;
            }

            temp = std::abs(cr[1]);
            if (temp > cmax) {
              cmax = temp;
              icmax = 1;
            }

            temp = std::abs(cr[2]);
            if (temp > cmax) {
              cmax = temp;
              icmax = 2;
            }

            temp = std::abs(cr[3]) + cr21;
            if (temp > cmax) {
              cmax = temp;
              icmax = 3;
            }

            if (cmax < smin) {
              double cr22;
              double lr21;
              double ur12;
              cr22 = B[ib0 - 1];
              ur12 = B[ib0 + 5];
              cr21 = B[ib0 + 6];
              lr21 = std::fmax(std::abs(cr22) + std::abs(ur12), std::abs(B[ib0])
                               + std::abs(cr21));
              if ((smin < 1.0) && (lr21 > 1.0) && (lr21 >
                   2.2471164185778949E+307 * smin)) {
                scale = 1.0 / lr21;
              }

              temp = scale / smin;
              X[0] = temp * cr22;
              X[1] = temp * B[ib0];
              X[2] = temp * ur12;
              X[3] = temp * cr21;
              xnorm = temp * lr21;
            } else {
              double bi1;
              double bi2;
              double br1;
              double br2;
              double cr22;
              double lr21;
              double ui11r;
              double ui12;
              double ui12s;
              double ur11r;
              double ur12;
              double ur12s;
              int b_cr21_tmp;
              int cr21_tmp;
              int ur12_tmp;
              cr21_tmp = icmax << 2;
              b_cr21_tmp = ipivot[cr21_tmp + 1] - 1;
              cr21 = cr[b_cr21_tmp];
              ur12_tmp = ipivot[cr21_tmp + 2] - 1;
              ur12 = cr[ur12_tmp];
              ui12 = ci[ur12_tmp];
              cr21_tmp = ipivot[cr21_tmp + 3] - 1;
              cr22 = cr[cr21_tmp];
              if ((icmax + 1 == 1) || (icmax + 1 == 4)) {
                if (std::abs(cr[icmax]) > std::abs(ci[icmax])) {
                  temp = ci[icmax] / cr[icmax];
                  ur11r = 1.0 / (cr[icmax] * (temp * temp + 1.0));
                  ui11r = -temp * ur11r;
                } else {
                  temp = cr[icmax] / ci[icmax];
                  ui11r = -1.0 / (ci[icmax] * (temp * temp + 1.0));
                  ur11r = -temp * ui11r;
                }

                lr21 = cr21 * ur11r;
                cr21 *= ui11r;
                ur12s = ur12 * ur11r;
                ui12s = ur12 * ui11r;
                cr22 -= ur12 * lr21;
                temp = ci[cr21_tmp] - ur12 * cr21;
              } else {
                ur11r = 1.0 / cr[icmax];
                ui11r = 0.0;
                lr21 = cr21 * ur11r;
                cr21 = ci[b_cr21_tmp] * ur11r;
                ur12s = ur12 * ur11r;
                ui12s = ui12 * ur11r;
                cr22 = (cr22 - ur12 * lr21) + ui12 * cr21;
                temp = -ur12 * cr21 - ui12 * lr21;
              }

              ur12 = std::abs(cr22) + std::abs(temp);
              if (ur12 < smin) {
                cr22 = smin;
                temp = 0.0;
              }

              if ((icmax + 1 == 2) || (icmax + 1 == 4)) {
                br2 = B[ib0 - 1];
                br1 = B[ib0];
                bi2 = B[ib0 + 5];
                bi1 = B[ib0 + 6];
              } else {
                br1 = B[ib0 - 1];
                br2 = B[ib0];
                bi1 = B[ib0 + 5];
                bi2 = B[ib0 + 6];
              }

              br2 = (br2 - lr21 * br1) + cr21 * bi1;
              bi2 = (bi2 - cr21 * br1) - lr21 * bi1;
              lr21 = std::fmax((std::abs(br1) + std::abs(bi1)) * (ur12 * (std::
                abs(ur11r) + std::abs(ui11r))), std::abs(br2) + std::abs(bi2));
              if ((lr21 > 1.0) && (ur12 < 1.0) && (lr21 >=
                   2.2471164185778949E+307 * ur12)) {
                scale = 1.0 / lr21;
                br1 *= scale;
                bi1 *= scale;
                br2 *= scale;
                bi2 *= scale;
              }

              cr22 = xdladiv(br2, bi2, cr22, temp, ui12);
              temp = ((ur11r * br1 - ui11r * bi1) - ur12s * cr22) + ui12s * ui12;
              cr21 = ((ui11r * br1 + ur11r * bi1) - ui12s * cr22) - ur12s * ui12;
              if ((icmax + 1 == 3) || (icmax + 1 == 4)) {
                X[0] = cr22;
                X[1] = temp;
                X[2] = ui12;
                X[3] = cr21;
              } else {
                X[0] = temp;
                X[1] = cr22;
                X[2] = cr21;
                X[3] = ui12;
              }

              xnorm = std::fmax(std::abs(temp) + std::abs(cr21), std::abs(cr22)
                                + std::abs(ui12));
              if ((xnorm > 1.0) && (cmax > 1.0) && (xnorm >
                   2.2471164185778949E+307 / cmax)) {
                temp = cmax / 2.2471164185778949E+307;
                X[0] *= temp;
                X[1] *= temp;
                X[2] *= temp;
                X[3] *= temp;
                xnorm *= temp;
                scale *= temp;
              }
            }
          }
        }

        return scale;
      }

      //
      // Arguments    : double &a
      //                double &b
      //                double &c
      //                double &d
      //                double &rt1i
      //                double &rt2r
      //                double &rt2i
      //                double &cs
      //                double &sn
      // Return Type  : double
      //
      static double xdlanv2(double &a, double &b, double &c, double &d, double
                            &rt1i, double &rt2r, double &rt2i, double &cs,
                            double &sn)
      {
        double rt1r;
        if (c == 0.0) {
          cs = 1.0;
          sn = 0.0;
        } else if (b == 0.0) {
          double temp;
          cs = 0.0;
          sn = 1.0;
          temp = d;
          d = a;
          a = temp;
          b = -c;
          c = 0.0;
        } else {
          double temp;
          temp = a - d;
          if ((temp == 0.0) && ((b < 0.0) != (c < 0.0))) {
            cs = 1.0;
            sn = 0.0;
          } else {
            double bcmax;
            double bcmis;
            double p;
            double scale;
            double z;
            int count;
            int i;
            p = 0.5 * temp;
            bcmis = std::abs(b);
            scale = std::abs(c);
            bcmax = std::fmax(bcmis, scale);
            if (!(b < 0.0)) {
              count = 1;
            } else {
              count = -1;
            }

            if (!(c < 0.0)) {
              i = 1;
            } else {
              i = -1;
            }

            bcmis = std::fmin(bcmis, scale) * static_cast<double>(count) *
              static_cast<double>(i);
            scale = std::fmax(std::abs(p), bcmax);
            z = p / scale * p + bcmax / scale * bcmis;
            if (z >= 8.8817841970012523E-16) {
              double tau;
              a = std::sqrt(scale) * std::sqrt(z);
              if (p < 0.0) {
                a = -a;
              }

              z = p + a;
              a = d + z;
              d -= bcmax / z * bcmis;
              tau = rt_hypotd_snf(c, z);
              cs = z / tau;
              sn = c / tau;
              b -= c;
              c = 0.0;
            } else {
              double tau;
              bcmis = b + c;
              scale = std::fmax(std::abs(temp), std::abs(bcmis));
              count = 0;
              while ((scale >= 7.4428285367870146E+137) && (count <= 20)) {
                bcmis *= 1.3435752215134178E-138;
                temp *= 1.3435752215134178E-138;
                scale = std::fmax(std::abs(temp), std::abs(bcmis));
                count++;
              }

              while ((scale <= 1.3435752215134178E-138) && (count <= 20)) {
                bcmis *= 7.4428285367870146E+137;
                temp *= 7.4428285367870146E+137;
                scale = std::fmax(std::abs(temp), std::abs(bcmis));
                count++;
              }

              tau = rt_hypotd_snf(bcmis, temp);
              cs = std::sqrt(0.5 * (std::abs(bcmis) / tau + 1.0));
              if (!(bcmis < 0.0)) {
                count = 1;
              } else {
                count = -1;
              }

              sn = -(0.5 * temp / (tau * cs)) * static_cast<double>(count);
              bcmax = a * cs + b * sn;
              scale = -a * sn + b * cs;
              z = c * cs + d * sn;
              bcmis = -c * sn + d * cs;
              b = scale * cs + bcmis * sn;
              c = -bcmax * sn + z * cs;
              temp = 0.5 * ((bcmax * cs + z * sn) + (-scale * sn + bcmis * cs));
              a = temp;
              d = temp;
              if (c != 0.0) {
                if (b != 0.0) {
                  if ((b < 0.0) == (c < 0.0)) {
                    bcmis = std::sqrt(std::abs(b));
                    scale = std::sqrt(std::abs(c));
                    a = bcmis * scale;
                    if (!(c < 0.0)) {
                      p = a;
                    } else {
                      p = -a;
                    }

                    tau = 1.0 / std::sqrt(std::abs(b + c));
                    a = temp + p;
                    d = temp - p;
                    b -= c;
                    c = 0.0;
                    bcmax = bcmis * tau;
                    bcmis = scale * tau;
                    temp = cs * bcmax - sn * bcmis;
                    sn = cs * bcmis + sn * bcmax;
                    cs = temp;
                  }
                } else {
                  b = -c;
                  c = 0.0;
                  temp = cs;
                  cs = -sn;
                  sn = temp;
                }
              }
            }
          }
        }

        rt1r = a;
        rt2r = d;
        if (c == 0.0) {
          rt1i = 0.0;
          rt2i = 0.0;
        } else {
          rt1i = std::sqrt(std::abs(b)) * std::sqrt(std::abs(c));
          rt2i = -rt1i;
        }

        return rt1r;
      }

      //
      // Arguments    : const double T[36]
      //                double vr[36]
      // Return Type  : void
      //
      static void xdtrevc3(const double T[36], double vr[36])
      {
        double work[18];
        double x[4];
        double emax;
        int ip;
        int iyend;
        int j;
        std::memset(&work[0], 0, 18U * sizeof(double));
        x[0] = 0.0;
        x[1] = 0.0;
        x[2] = 0.0;
        x[3] = 0.0;
        work[0] = 0.0;
        for (j = 0; j < 5; j++) {
          work[j + 1] = 0.0;
          for (iyend = 0; iyend <= j; iyend++) {
            work[j + 1] += std::abs(T[iyend + 6 * (j + 1)]);
          }
        }

        ip = 0;
        for (int ki{5}; ki >= 0; ki--) {
          if (ip == -1) {
            ip = 1;
          } else {
            double smin;
            double wi;
            double wr_tmp;
            if ((ki + 1 == 1) || (T[ki + 6 * (ki - 1)] == 0.0)) {
              ip = 0;
            } else {
              ip = -1;
            }

            iyend = ki + 6 * ki;
            wr_tmp = T[iyend];
            wi = 0.0;
            if (ip != 0) {
              wi = std::sqrt(std::abs(T[ki + 6 * (ki - 1)])) * std::sqrt(std::
                abs(T[iyend - 1]));
            }

            smin = std::fmax(2.2204460492503131E-16 * (std::abs(wr_tmp) + wi),
                             6.0125050800269183E-292);
            if (ip == 0) {
              double scale;
              int i;
              work[ki + 12] = 1.0;
              for (int k{0}; k < ki; k++) {
                work[k + 12] = -T[k + 6 * ki];
              }

              j = ki - 1;
              int exitg1;
              do {
                exitg1 = 0;
                if (j + 1 >= 1) {
                  boolean_T guard1;
                  guard1 = false;
                  if (j + 1 == 1) {
                    guard1 = true;
                  } else {
                    int i1;
                    i = 6 * (j - 1);
                    i1 = j + i;
                    if (T[i1] == 0.0) {
                      guard1 = true;
                    } else {
                      scale = xdlaln2(2, 1, smin, T, i1, work, j + 12, wr_tmp,
                                      0.0, x, emax);
                      if ((emax > 1.0) && (std::fmax(work[j - 1], work[j]) >
                                           1.6632002579455995E+291 / emax)) {
                        x[0] /= emax;
                        x[1] /= emax;
                        scale /= emax;
                      }

                      if (scale != 1.0) {
                        i1 = ki + 13;
                        for (int k{13}; k <= i1; k++) {
                          work[k - 1] *= scale;
                        }
                      }

                      work[j + 11] = x[0];
                      work[j + 12] = x[1];
                      blas::xaxpy(j - 1, -x[0], T, i + 1, work);
                      blas::xaxpy(j - 1, -x[1], T, j * 6 + 1, work);
                      j -= 2;
                    }
                  }

                  if (guard1) {
                    scale = xdlaln2(1, 1, smin, T, (j * 6 + j) + 1, work, j + 13,
                                    wr_tmp, 0.0, x, emax);
                    if ((emax > 1.0) && (work[j] > 1.6632002579455995E+291 /
                                         emax)) {
                      x[0] /= emax;
                      scale /= emax;
                    }

                    if (scale != 1.0) {
                      i = ki + 13;
                      for (int k{13}; k <= i; k++) {
                        work[k - 1] *= scale;
                      }
                    }

                    work[j + 12] = x[0];
                    blas::xaxpy(j, -x[0], T, j * 6 + 1, work);
                    j--;
                  }
                } else {
                  exitg1 = 1;
                }
              } while (exitg1 == 0);

              if (ki + 1 > 1) {
                blas::xgemv(ki, work, work[ki + 12], vr, ki * 6 + 1);
              }

              j = ki * 6;
              iyend = -1;
              emax = std::abs(vr[j]);
              for (int k{0}; k < 5; k++) {
                scale = std::abs(vr[(j + k) + 1]);
                if (scale > emax) {
                  iyend = k;
                  emax = scale;
                }
              }

              emax = 1.0 / std::abs(vr[(iyend + 6 * ki) + 1]);
              i = j + 6;
              for (int k{j + 1}; k <= i; k++) {
                vr[k - 1] *= emax;
              }
            } else {
              double scale;
              int i;
              int i1;
              int ix;
              int ix0;
              emax = T[iyend - 1];
              ix0 = 6 * (ki - 1);
              scale = T[ki + ix0];
              if (std::abs(emax) >= std::abs(scale)) {
                work[ki + 5] = 1.0;
                work[ki + 12] = wi / emax;
              } else {
                work[ki + 5] = -wi / scale;
                work[ki + 12] = 1.0;
              }

              work[ki + 6] = 0.0;
              work[ki + 11] = 0.0;
              for (int k{0}; k <= ki - 2; k++) {
                work[k + 6] = -work[ki + 5] * T[k + ix0];
                work[k + 12] = -work[ki + 12] * T[k + 6 * ki];
              }

              j = ki - 2;
              int exitg1;
              do {
                exitg1 = 0;
                if (j + 1 >= 1) {
                  boolean_T guard1;
                  guard1 = false;
                  if (j + 1 == 1) {
                    guard1 = true;
                  } else {
                    i = 6 * (j - 1);
                    i1 = j + i;
                    if (T[i1] == 0.0) {
                      guard1 = true;
                    } else {
                      scale = xdlaln2(2, 2, smin, T, i1, work, j + 6, wr_tmp, wi,
                                      x, emax);
                      if ((emax > 1.0) && (std::fmax(work[j - 1], work[j]) >
                                           1.6632002579455995E+291 / emax)) {
                        emax = 1.0 / emax;
                        x[0] *= emax;
                        x[2] *= emax;
                        x[1] *= emax;
                        x[3] *= emax;
                        scale *= emax;
                      }

                      if (scale != 1.0) {
                        i1 = ki + 7;
                        for (int k{7}; k <= i1; k++) {
                          work[k - 1] *= scale;
                        }

                        i1 = ki + 13;
                        for (int k{13}; k <= i1; k++) {
                          work[k - 1] *= scale;
                        }
                      }

                      work[j + 5] = x[0];
                      work[j + 6] = x[1];
                      work[j + 11] = x[2];
                      work[j + 12] = x[3];
                      if ((j - 1 >= 1) && (!(-x[0] == 0.0))) {
                        i1 = j - 2;
                        for (int k{0}; k <= i1; k++) {
                          work[k + 6] += -x[0] * T[i + k];
                        }
                      }

                      if ((j - 1 >= 1) && (!(-x[1] == 0.0))) {
                        ix = j * 6;
                        i1 = j - 2;
                        for (int k{0}; k <= i1; k++) {
                          work[k + 6] += -x[1] * T[ix + k];
                        }
                      }

                      blas::xaxpy(j - 1, -x[2], T, i + 1, work);
                      blas::xaxpy(j - 1, -x[3], T, j * 6 + 1, work);
                      j -= 2;
                    }
                  }

                  if (guard1) {
                    scale = xdlaln2(1, 2, smin, T, (j * 6 + j) + 1, work, j + 7,
                                    wr_tmp, wi, x, emax);
                    if ((emax > 1.0) && (work[j] > 1.6632002579455995E+291 /
                                         emax)) {
                      x[0] /= emax;
                      x[2] /= emax;
                      scale /= emax;
                    }

                    if (scale != 1.0) {
                      i = ki + 7;
                      for (int k{7}; k <= i; k++) {
                        work[k - 1] *= scale;
                      }

                      i = ki + 13;
                      for (int k{13}; k <= i; k++) {
                        work[k - 1] *= scale;
                      }
                    }

                    work[j + 6] = x[0];
                    work[j + 12] = x[2];
                    if ((j >= 1) && (!(-x[0] == 0.0))) {
                      ix = j * 6;
                      i = j - 1;
                      for (int k{0}; k <= i; k++) {
                        work[k + 6] += -x[0] * T[ix + k];
                      }
                    }

                    blas::xaxpy(j, -x[2], T, j * 6 + 1, work);
                    j--;
                  }
                } else {
                  exitg1 = 1;
                }
              } while (exitg1 == 0);

              if (ki + 1 > 2) {
                iyend = ix0 + 6;
                emax = work[ki + 5];
                if (emax != 1.0) {
                  if (emax == 0.0) {
                    if (ix0 + 1 <= iyend) {
                      std::memset(&vr[ix0], 0, static_cast<unsigned int>(iyend -
                        ix0) * sizeof(double));
                    }
                  } else {
                    for (j = ix0 + 1; j <= iyend; j++) {
                      vr[j - 1] *= emax;
                    }
                  }
                }

                ix = 6;
                i = 6 * (ki - 2) + 1;
                for (j = 1; j <= i; j += 6) {
                  i1 = j + 5;
                  for (int k{j}; k <= i1; k++) {
                    iyend = (ix0 + k) - j;
                    vr[iyend] += vr[k - 1] * work[ix];
                  }

                  ix++;
                }

                blas::xgemv(ki - 1, work, work[ki + 12], vr, ki * 6 + 1);
              } else {
                i = ix0 + 6;
                for (int k{ix0 + 1}; k <= i; k++) {
                  vr[k - 1] *= work[6];
                }

                iyend = ki * 6;
                i = iyend + 6;
                for (int k{iyend + 1}; k <= i; k++) {
                  vr[k - 1] *= work[ki + 12];
                }
              }

              emax = 0.0;
              for (int k{0}; k < 6; k++) {
                emax = std::fmax(emax, std::abs(vr[k + ix0]) + std::abs(vr[k + 6
                  * ki]));
              }

              emax = 1.0 / emax;
              i = ix0 + 6;
              for (int k{ix0 + 1}; k <= i; k++) {
                vr[k - 1] *= emax;
              }

              ix0 = ki * 6;
              i = ix0 + 6;
              for (int k{ix0 + 1}; k <= i; k++) {
                vr[k - 1] *= emax;
              }
            }
          }
        }
      }

      //
      // Arguments    : double A[36]
      //                int &ihi
      //                double scale[6]
      // Return Type  : int
      //
      static int xzgebal(double A[36], int &ihi, double scale[6])
      {
        double b_scale;
        int b_i;
        int b_ix0_tmp;
        int exitg5;
        int i;
        int ilo;
        int ira;
        int ix0_tmp;
        int kend;
        boolean_T converged;
        boolean_T notdone;
        for (i = 0; i < 6; i++) {
          scale[i] = 1.0;
        }

        ilo = 1;
        ihi = 6;
        notdone = true;
        do {
          exitg5 = 0;
          if (notdone) {
            int exitg4;
            int j;
            notdone = false;
            j = ihi;
            do {
              exitg4 = 0;
              if (j > 0) {
                boolean_T exitg6;
                converged = false;
                i = 0;
                exitg6 = false;
                while ((!exitg6) && (i <= static_cast<unsigned char>(ihi) - 1))
                {
                  if ((i + 1 == j) || (!(A[(j + 6 * i) - 1] != 0.0))) {
                    i++;
                  } else {
                    converged = true;
                    exitg6 = true;
                  }
                }

                if (converged) {
                  j--;
                } else {
                  scale[ihi - 1] = j;
                  if (j != ihi) {
                    ira = (j - 1) * 6;
                    kend = (ihi - 1) * 6;
                    b_i = static_cast<unsigned char>(ihi);
                    for (int k{0}; k < b_i; k++) {
                      b_ix0_tmp = ira + k;
                      b_scale = A[b_ix0_tmp];
                      ix0_tmp = kend + k;
                      A[b_ix0_tmp] = A[ix0_tmp];
                      A[ix0_tmp] = b_scale;
                    }

                    for (int k{0}; k < 6; k++) {
                      b_ix0_tmp = (j + k * 6) - 1;
                      b_scale = A[b_ix0_tmp];
                      b_i = (ihi + k * 6) - 1;
                      A[b_ix0_tmp] = A[b_i];
                      A[b_i] = b_scale;
                    }
                  }

                  exitg4 = 1;
                }
              } else {
                exitg4 = 2;
              }
            } while (exitg4 == 0);

            if (exitg4 == 1) {
              if (ihi == 1) {
                ilo = 1;
                ihi = 1;
                exitg5 = 1;
              } else {
                ihi--;
                notdone = true;
              }
            }
          } else {
            notdone = true;
            while (notdone) {
              int j;
              boolean_T exitg6;
              notdone = false;
              j = ilo;
              exitg6 = false;
              while ((!exitg6) && (j <= ihi)) {
                boolean_T exitg7;
                converged = false;
                i = ilo;
                exitg7 = false;
                while ((!exitg7) && (i <= ihi)) {
                  if ((i == j) || (!(A[(i + 6 * (j - 1)) - 1] != 0.0))) {
                    i++;
                  } else {
                    converged = true;
                    exitg7 = true;
                  }
                }

                if (converged) {
                  j++;
                } else {
                  scale[ilo - 1] = j;
                  if (j != ilo) {
                    ira = (j - 1) * 6;
                    kend = (ilo - 1) * 6;
                    b_i = static_cast<unsigned char>(ihi);
                    for (int k{0}; k < b_i; k++) {
                      b_ix0_tmp = ira + k;
                      b_scale = A[b_ix0_tmp];
                      ix0_tmp = kend + k;
                      A[b_ix0_tmp] = A[ix0_tmp];
                      A[ix0_tmp] = b_scale;
                    }

                    ira = (kend + j) - 1;
                    kend = (kend + ilo) - 1;
                    b_i = static_cast<unsigned char>(7 - ilo);
                    for (int k{0}; k < b_i; k++) {
                      b_ix0_tmp = ira + k * 6;
                      b_scale = A[b_ix0_tmp];
                      ix0_tmp = kend + k * 6;
                      A[b_ix0_tmp] = A[ix0_tmp];
                      A[ix0_tmp] = b_scale;
                    }
                  }

                  ilo++;
                  notdone = true;
                  exitg6 = true;
                }
              }
            }

            converged = false;
            exitg5 = 2;
          }
        } while (exitg5 == 0);

        if (exitg5 != 1) {
          boolean_T exitg3;
          exitg3 = false;
          while ((!exitg3) && (!converged)) {
            int exitg2;
            converged = true;
            i = ilo - 1;
            do {
              exitg2 = 0;
              if (i + 1 <= ihi) {
                double absxk;
                double c;
                double ca;
                double r;
                double t;
                kend = (ihi - ilo) + 1;
                c = blas::xnrm2(kend, A, i * 6 + ilo);
                ix0_tmp = (ilo - 1) * 6 + i;
                r = 0.0;
                if (kend >= 1) {
                  if (kend == 1) {
                    r = std::abs(A[ix0_tmp]);
                  } else {
                    b_scale = 3.3121686421112381E-170;
                    kend = (ix0_tmp + (kend - 1) * 6) + 1;
                    for (int k{ix0_tmp + 1}; k <= kend; k += 6) {
                      absxk = std::abs(A[k - 1]);
                      if (absxk > b_scale) {
                        t = b_scale / absxk;
                        r = r * t * t + 1.0;
                        b_scale = absxk;
                      } else {
                        t = absxk / b_scale;
                        r += t * t;
                      }
                    }

                    r = b_scale * std::sqrt(r);
                  }
                }

                b_ix0_tmp = i * 6;
                kend = 1;
                if (ihi > 1) {
                  b_scale = std::abs(A[b_ix0_tmp]);
                  for (int k{2}; k <= ihi; k++) {
                    t = std::abs(A[(b_ix0_tmp + k) - 1]);
                    if (t > b_scale) {
                      kend = k;
                      b_scale = t;
                    }
                  }
                }

                ca = std::abs(A[(kend + 6 * i) - 1]);
                kend = 7 - ilo;
                if (7 - ilo < 1) {
                  ira = 0;
                } else {
                  ira = 1;
                  if (7 - ilo > 1) {
                    b_scale = std::abs(A[ix0_tmp]);
                    for (int k{2}; k <= kend; k++) {
                      t = std::abs(A[ix0_tmp + (k - 1) * 6]);
                      if (t > b_scale) {
                        ira = k;
                        b_scale = t;
                      }
                    }
                  }
                }

                b_scale = std::abs(A[i + 6 * ((ira + ilo) - 2)]);
                if ((c == 0.0) || (r == 0.0)) {
                  i++;
                } else {
                  double f;
                  int exitg1;
                  absxk = r / 2.0;
                  f = 1.0;
                  t = c + r;
                  do {
                    exitg1 = 0;
                    if ((c < absxk) && (std::fmax(f, std::fmax(c, ca)) <
                                        4.9896007738368E+291) && (std::fmin(r,
                          std::fmin(absxk, b_scale)) > 2.0041683600089728E-292))
                    {
                      if (std::isnan(((((c + f) + ca) + r) + absxk) + b_scale))
                      {
                        exitg1 = 1;
                      } else {
                        f *= 2.0;
                        c *= 2.0;
                        ca *= 2.0;
                        r /= 2.0;
                        absxk /= 2.0;
                        b_scale /= 2.0;
                      }
                    } else {
                      absxk = c / 2.0;
                      while ((absxk >= r) && (std::fmax(r, b_scale) <
                                              4.9896007738368E+291) && (std::
                              fmin(std::fmin(f, c), std::fmin(absxk, ca)) >
                              2.0041683600089728E-292)) {
                        f /= 2.0;
                        c /= 2.0;
                        absxk /= 2.0;
                        ca /= 2.0;
                        r *= 2.0;
                        b_scale *= 2.0;
                      }

                      if ((!(c + r >= 0.95 * t)) && ((!(f < 1.0)) || (!(scale[i]
                             < 1.0)) || (!(f * scale[i] <=
                                           1.0020841800044864E-292))) && ((!(f >
                             1.0)) || (!(scale[i] > 1.0)) || (!(scale[i] >=
                             9.9792015476736E+291 / f)))) {
                        b_scale = 1.0 / f;
                        scale[i] *= f;
                        kend = ix0_tmp + 1;
                        b_i = (ix0_tmp + 6 * (6 - ilo)) + 1;
                        for (int k{kend}; k <= b_i; k += 6) {
                          A[k - 1] *= b_scale;
                        }

                        b_i = b_ix0_tmp + ihi;
                        for (int k{b_ix0_tmp + 1}; k <= b_i; k++) {
                          A[k - 1] *= f;
                        }

                        converged = false;
                      }

                      exitg1 = 2;
                    }
                  } while (exitg1 == 0);

                  if (exitg1 == 1) {
                    exitg2 = 2;
                  } else {
                    i++;
                  }
                }
              } else {
                exitg2 = 1;
              }
            } while (exitg2 == 0);

            if (exitg2 != 1) {
              exitg3 = true;
            }
          }
        }

        return ilo;
      }

      //
      // Arguments    : double a[36]
      //                int ilo
      //                int ihi
      //                double tau[5]
      // Return Type  : void
      //
      static void xzgehrd(double a[36], int ilo, int ihi, double tau[5])
      {
        double work[6];
        double alpha1;
        if ((ihi - ilo) + 1 > 1) {
          int i;
          i = static_cast<unsigned char>(ilo - 1);
          if (i - 1 >= 0) {
            std::memset(&tau[0], 0, static_cast<unsigned int>(i) * sizeof(double));
          }

          for (int b_i{ihi}; b_i < 6; b_i++) {
            tau[b_i - 1] = 0.0;
          }

          for (int b_i{0}; b_i < 6; b_i++) {
            work[b_i] = 0.0;
          }

          i = ihi - 1;
          for (int b_i{ilo}; b_i <= i; b_i++) {
            double d;
            int alpha1_tmp_tmp;
            int c_i;
            int i1;
            int ia;
            int ic0;
            int in;
            int lastc;
            int lastv;
            int n;
            c_i = (b_i - 1) * 6;
            in = b_i * 6;
            alpha1_tmp_tmp = b_i + c_i;
            alpha1 = a[alpha1_tmp_tmp];
            n = ihi - b_i;
            if (b_i + 2 <= 6) {
              i1 = b_i + 1;
            } else {
              i1 = 5;
            }

            d = xzlarfg(n, alpha1, a, (i1 + c_i) + 1);
            tau[b_i - 1] = d;
            a[alpha1_tmp_tmp] = 1.0;
            ic0 = in + 1;
            if (d != 0.0) {
              boolean_T exitg2;
              lastv = n;
              c_i = alpha1_tmp_tmp + n;
              while ((lastv > 0) && (a[c_i - 1] == 0.0)) {
                lastv--;
                c_i--;
              }

              lastc = ihi;
              exitg2 = false;
              while ((!exitg2) && (lastc > 0)) {
                int exitg1;
                c_i = in + lastc;
                ia = c_i;
                do {
                  exitg1 = 0;
                  if (ia <= c_i + (lastv - 1) * 6) {
                    if (a[ia - 1] != 0.0) {
                      exitg1 = 1;
                    } else {
                      ia += 6;
                    }
                  } else {
                    lastc--;
                    exitg1 = 2;
                  }
                } while (exitg1 == 0);

                if (exitg1 == 1) {
                  exitg2 = true;
                }
              }
            } else {
              lastv = 0;
              lastc = 0;
            }

            if (lastv > 0) {
              int i2;
              int jA;
              if (lastc != 0) {
                i1 = static_cast<unsigned char>(lastc);
                std::memset(&work[0], 0, static_cast<unsigned int>(i1) * sizeof
                            (double));
                c_i = alpha1_tmp_tmp;
                i1 = (in + 6 * (lastv - 1)) + 1;
                for (int iac{ic0}; iac <= i1; iac += 6) {
                  i2 = (iac + lastc) - 1;
                  for (ia = iac; ia <= i2; ia++) {
                    jA = ia - iac;
                    work[jA] += a[ia - 1] * a[c_i];
                  }

                  c_i++;
                }
              }

              d = -tau[b_i - 1];
              if (!(d == 0.0)) {
                jA = in;
                i1 = static_cast<unsigned char>(lastv);
                for (int iac{0}; iac < i1; iac++) {
                  double temp;
                  temp = a[alpha1_tmp_tmp + iac];
                  if (temp != 0.0) {
                    temp *= d;
                    i2 = jA + 1;
                    c_i = lastc + jA;
                    for (ic0 = i2; ic0 <= c_i; ic0++) {
                      a[ic0 - 1] += work[(ic0 - jA) - 1] * temp;
                    }
                  }

                  jA += 6;
                }
              }
            }

            xzlarf(n, 6 - b_i, alpha1_tmp_tmp + 1, tau[b_i - 1], a, (b_i + in) +
                   1, work);
            a[alpha1_tmp_tmp] = alpha1;
          }
        }
      }

      //
      // Arguments    : double A[36]
      //                int ipiv[6]
      // Return Type  : int
      //
      static int xzgetrf(double A[36], int ipiv[6])
      {
        int i;
        int info;
        for (i = 0; i < 6; i++) {
          ipiv[i] = i + 1;
        }

        info = 0;
        for (int j{0}; j < 5; j++) {
          double smax;
          int a;
          int b_tmp;
          int jA;
          int jp1j;
          int mmj_tmp;
          mmj_tmp = 4 - j;
          b_tmp = j * 7;
          jp1j = b_tmp + 2;
          jA = 6 - j;
          a = 0;
          smax = std::abs(A[b_tmp]);
          for (int k{2}; k <= jA; k++) {
            double s;
            s = std::abs(A[(b_tmp + k) - 1]);
            if (s > smax) {
              a = k - 1;
              smax = s;
            }
          }

          if (A[b_tmp + a] != 0.0) {
            if (a != 0) {
              jA = j + a;
              ipiv[j] = jA + 1;
              for (int k{0}; k < 6; k++) {
                a = j + k * 6;
                smax = A[a];
                i = jA + k * 6;
                A[a] = A[i];
                A[i] = smax;
              }
            }

            i = (b_tmp - j) + 6;
            for (jA = jp1j; jA <= i; jA++) {
              A[jA - 1] /= A[b_tmp];
            }
          } else {
            info = j + 1;
          }

          jA = b_tmp;
          for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
            smax = A[(b_tmp + jp1j * 6) + 6];
            if (smax != 0.0) {
              i = jA + 8;
              a = (jA - j) + 12;
              for (int k{i}; k <= a; k++) {
                A[k - 1] += A[((b_tmp + k) - jA) - 7] * -smax;
              }
            }

            jA += 6;
          }
        }

        if ((info == 0) && (!(A[35] != 0.0))) {
          info = 6;
        }

        return info;
      }

      //
      // Arguments    : int m
      //                int n
      //                int iv0
      //                double tau
      //                double C[36]
      //                int ic0
      //                double work[6]
      // Return Type  : void
      //
      static void xzlarf(int m, int n, int iv0, double tau, double C[36], int
                         ic0, double work[6])
      {
        int i;
        int ia;
        int lastc;
        int lastv;
        if (tau != 0.0) {
          boolean_T exitg2;
          lastv = m;
          i = iv0 + m;
          while ((lastv > 0) && (C[i - 2] == 0.0)) {
            lastv--;
            i--;
          }

          lastc = n - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            i = ic0 + lastc * 6;
            ia = i;
            do {
              exitg1 = 0;
              if (ia <= (i + lastv) - 1) {
                if (C[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = -1;
        }

        if (lastv > 0) {
          double c;
          int b_i;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              std::memset(&work[0], 0, static_cast<unsigned int>(lastc + 1) *
                          sizeof(double));
            }

            b_i = ic0 + 6 * lastc;
            for (int iac{ic0}; iac <= b_i; iac += 6) {
              c = 0.0;
              i = (iac + lastv) - 1;
              for (ia = iac; ia <= i; ia++) {
                c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
              }

              i = div_nde_s32_floor(iac - ic0);
              work[i] += c;
            }
          }

          if (!(-tau == 0.0)) {
            i = ic0;
            for (int iac{0}; iac <= lastc; iac++) {
              c = work[iac];
              if (c != 0.0) {
                c *= -tau;
                b_i = lastv + i;
                for (ia = i; ia < b_i; ia++) {
                  C[ia - 1] += C[((iv0 + ia) - i) - 1] * c;
                }
              }

              i += 6;
            }
          }
        }
      }

      //
      // Arguments    : int n
      //                double &alpha1
      //                double x[36]
      //                int ix0
      // Return Type  : double
      //
      static double xzlarfg(int n, double &alpha1, double x[36], int ix0)
      {
        double tau;
        tau = 0.0;
        if (n > 0) {
          double xnorm;
          xnorm = blas::xnrm2(n - 1, x, ix0);
          if (xnorm != 0.0) {
            double beta1;
            beta1 = rt_hypotd_snf(alpha1, xnorm);
            if (alpha1 >= 0.0) {
              beta1 = -beta1;
            }

            if (std::abs(beta1) < 1.0020841800044864E-292) {
              int i;
              int knt;
              knt = 0;
              i = (ix0 + n) - 2;
              do {
                knt++;
                for (int k{ix0}; k <= i; k++) {
                  x[k - 1] *= 9.9792015476736E+291;
                }

                beta1 *= 9.9792015476736E+291;
                alpha1 *= 9.9792015476736E+291;
              } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));

              beta1 = rt_hypotd_snf(alpha1, blas::xnrm2(n - 1, x, ix0));
              if (alpha1 >= 0.0) {
                beta1 = -beta1;
              }

              tau = (beta1 - alpha1) / beta1;
              xnorm = 1.0 / (alpha1 - beta1);
              for (int k{ix0}; k <= i; k++) {
                x[k - 1] *= xnorm;
              }

              for (int k{0}; k < knt; k++) {
                beta1 *= 1.0020841800044864E-292;
              }

              alpha1 = beta1;
            } else {
              int i;
              tau = (beta1 - alpha1) / beta1;
              xnorm = 1.0 / (alpha1 - beta1);
              i = (ix0 + n) - 2;
              for (int k{ix0}; k <= i; k++) {
                x[k - 1] *= xnorm;
              }

              alpha1 = beta1;
            }
          }
        }

        return tau;
      }

      //
      // Arguments    : int n
      //                double &alpha1
      //                double x[3]
      // Return Type  : double
      //
      static double xzlarfg(int n, double &alpha1, double x[3])
      {
        double tau;
        tau = 0.0;
        if (n > 0) {
          double xnorm;
          xnorm = blas::xnrm2(n - 1, x);
          if (xnorm != 0.0) {
            double beta1;
            beta1 = rt_hypotd_snf(alpha1, xnorm);
            if (alpha1 >= 0.0) {
              beta1 = -beta1;
            }

            if (std::abs(beta1) < 1.0020841800044864E-292) {
              int knt;
              knt = 0;
              do {
                knt++;
                for (int k{2}; k <= n; k++) {
                  x[k - 1] *= 9.9792015476736E+291;
                }

                beta1 *= 9.9792015476736E+291;
                alpha1 *= 9.9792015476736E+291;
              } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));

              beta1 = rt_hypotd_snf(alpha1, blas::xnrm2(n - 1, x));
              if (alpha1 >= 0.0) {
                beta1 = -beta1;
              }

              tau = (beta1 - alpha1) / beta1;
              xnorm = 1.0 / (alpha1 - beta1);
              for (int k{2}; k <= n; k++) {
                x[k - 1] *= xnorm;
              }

              for (int k{0}; k < knt; k++) {
                beta1 *= 1.0020841800044864E-292;
              }

              alpha1 = beta1;
            } else {
              tau = (beta1 - alpha1) / beta1;
              xnorm = 1.0 / (alpha1 - beta1);
              for (int k{2}; k <= n; k++) {
                x[k - 1] *= xnorm;
              }

              alpha1 = beta1;
            }
          }
        }

        return tau;
      }

      //
      // Arguments    : double f
      //                double g
      //                double &sn
      //                double &r
      // Return Type  : double
      //
      static double xzlartg(double f, double g, double &sn, double &r)
      {
        double cs;
        double f1;
        f1 = std::abs(f);
        r = std::abs(g);
        if (g == 0.0) {
          cs = 1.0;
          sn = 0.0;
          r = f;
        } else if (f == 0.0) {
          cs = 0.0;
          if (g >= 0.0) {
            sn = 1.0;
          } else {
            sn = -1.0;
          }
        } else if ((f1 > 1.4916681462400413E-154) && (f1 <
                    4.7403759540545887E+153) && (r > 1.4916681462400413E-154) &&
                   (r < 4.7403759540545887E+153)) {
          r = std::sqrt(f * f + g * g);
          cs = f1 / r;
          if (!(f >= 0.0)) {
            r = -r;
          }

          sn = g / r;
        } else {
          double gs;
          double u;
          u = std::fmin(4.49423283715579E+307, std::fmax(2.2250738585072014E-308,
            std::fmax(f1, r)));
          f1 = f / u;
          gs = g / u;
          r = std::sqrt(f1 * f1 + gs * gs);
          cs = std::abs(f1) / r;
          if (!(f >= 0.0)) {
            r = -r;
          }

          sn = gs / r;
          r *= u;
        }

        return cs;
      }

      //
      // Arguments    : double cfrom
      //                double cto
      //                double A[36]
      // Return Type  : void
      //
      static void xzlascl(double cfrom, double cto, double A[36])
      {
        double cfromc;
        double ctoc;
        boolean_T notdone;
        cfromc = cfrom;
        ctoc = cto;
        notdone = true;
        while (notdone) {
          double cfrom1;
          double cto1;
          double mul;
          cfrom1 = cfromc * 2.0041683600089728E-292;
          cto1 = ctoc / 4.9896007738368E+291;
          if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
            mul = 2.0041683600089728E-292;
            cfromc = cfrom1;
          } else if (std::abs(cto1) > std::abs(cfromc)) {
            mul = 4.9896007738368E+291;
            ctoc = cto1;
          } else {
            mul = ctoc / cfromc;
            notdone = false;
          }

          for (int i{0}; i < 36; i++) {
            A[i] *= mul;
          }
        }
      }

      //
      // Arguments    : double cfrom
      //                double cto
      //                int m
      //                double A[6]
      //                int iA0
      // Return Type  : void
      //
      static void xzlascl(double cfrom, double cto, int m, double A[6], int iA0)
      {
        double cfromc;
        double ctoc;
        boolean_T notdone;
        cfromc = cfrom;
        ctoc = cto;
        notdone = true;
        while (notdone) {
          double cfrom1;
          double cto1;
          double mul;
          cfrom1 = cfromc * 2.0041683600089728E-292;
          cto1 = ctoc / 4.9896007738368E+291;
          if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
            mul = 2.0041683600089728E-292;
            cfromc = cfrom1;
          } else if (std::abs(cto1) > std::abs(cfromc)) {
            mul = 4.9896007738368E+291;
            ctoc = cto1;
          } else {
            mul = ctoc / cfromc;
            notdone = false;
          }

          for (int i{0}; i < m; i++) {
            int b_i;
            b_i = (iA0 + i) - 1;
            A[b_i] *= mul;
          }
        }
      }

      //
      // Arguments    : double d[6]
      //                double e[5]
      //                double z[36]
      // Return Type  : int
      //
      static int xzsteqr(double d[6], double e[5], double z[36])
      {
        double work[10];
        double c;
        double r;
        double s;
        double tst;
        int info;
        int jtot;
        int l1;
        info = 0;
        std::memset(&work[0], 0, 10U * sizeof(double));
        jtot = 0;
        l1 = 1;
        int exitg1;
        do {
          exitg1 = 0;
          if (l1 > 6) {
            for (l1 = 0; l1 < 5; l1++) {
              double p;
              int iscale;
              int m;
              m = l1;
              p = d[l1];
              for (iscale = l1 + 2; iscale < 7; iscale++) {
                c = d[iscale - 1];
                if (c < p) {
                  m = iscale - 1;
                  p = c;
                }
              }

              if (m != l1) {
                int iy;
                d[m] = d[l1];
                d[l1] = p;
                iscale = l1 * 6;
                iy = m * 6;
                for (m = 0; m < 6; m++) {
                  int b_i;
                  jtot = iscale + m;
                  tst = z[jtot];
                  b_i = iy + m;
                  z[jtot] = z[b_i];
                  z[b_i] = tst;
                }
              }
            }

            exitg1 = 1;
          } else {
            int l;
            int lend;
            int lendsv;
            int lsv;
            int m;
            boolean_T exitg2;
            if (l1 > 1) {
              e[l1 - 2] = 0.0;
            }

            m = l1;
            exitg2 = false;
            while ((!exitg2) && (m < 6)) {
              tst = std::abs(e[m - 1]);
              if (tst == 0.0) {
                exitg2 = true;
              } else if (tst <= std::sqrt(std::abs(d[m - 1])) * std::sqrt(std::
                          abs(d[m])) * 2.2204460492503131E-16) {
                e[m - 1] = 0.0;
                exitg2 = true;
              } else {
                m++;
              }
            }

            l = l1 - 1;
            lsv = l1;
            lend = m;
            lendsv = m;
            l1 = m + 1;
            if (m != l + 1) {
              double anorm;
              int i;
              int iscale;
              int iy;
              iy = m - l;
              if (iy <= 0) {
                anorm = 0.0;
              } else {
                anorm = std::abs(d[(l + iy) - 1]);
                i = 0;
                exitg2 = false;
                while ((!exitg2) && (i <= iy - 2)) {
                  iscale = l + i;
                  tst = std::abs(d[iscale]);
                  if (std::isnan(tst)) {
                    anorm = rtNaN;
                    exitg2 = true;
                  } else {
                    if (tst > anorm) {
                      anorm = tst;
                    }

                    tst = std::abs(e[iscale]);
                    if (std::isnan(tst)) {
                      anorm = rtNaN;
                      exitg2 = true;
                    } else {
                      if (tst > anorm) {
                        anorm = tst;
                      }

                      i++;
                    }
                  }
                }
              }

              iscale = 0;
              if (!(anorm == 0.0)) {
                if (std::isinf(anorm) || std::isnan(anorm)) {
                  for (i = 0; i < 6; i++) {
                    d[i] = rtNaN;
                  }

                  for (int b_i{0}; b_i < 36; b_i++) {
                    z[b_i] = rtNaN;
                  }

                  exitg1 = 1;
                } else {
                  int b_i;
                  if (anorm > 2.2346346549904327E+153) {
                    iscale = 1;
                    xzlascl(anorm, 2.2346346549904327E+153, iy, d, l + 1);
                    b_xzlascl(anorm, 2.2346346549904327E+153, iy - 1, e, l + 1);
                  } else if (anorm < 3.02546243347603E-123) {
                    iscale = 2;
                    xzlascl(anorm, 3.02546243347603E-123, iy, d, l + 1);
                    b_xzlascl(anorm, 3.02546243347603E-123, iy - 1, e, l + 1);
                  }

                  if (std::abs(d[m - 1]) < std::abs(d[l])) {
                    lend = lsv;
                    l = m - 1;
                  }

                  if (lend > l + 1) {
                    int exitg4;
                    do {
                      exitg4 = 0;
                      if (l + 1 != lend) {
                        m = l + 1;
                        exitg2 = false;
                        while ((!exitg2) && (m < lend)) {
                          tst = std::abs(e[m - 1]);
                          if (tst * tst <= 4.9303806576313238E-32 * std::abs(d[m
                               - 1]) * std::abs(d[m]) + 2.2250738585072014E-308)
                          {
                            exitg2 = true;
                          } else {
                            m++;
                          }
                        }
                      } else {
                        m = lend;
                      }

                      if (m < lend) {
                        e[m - 1] = 0.0;
                      }

                      if (m == l + 1) {
                        l++;
                        if (l + 1 > lend) {
                          exitg4 = 1;
                        }
                      } else if (m == l + 2) {
                        d[l] = xdlaev2(d[l], e[l], d[l + 1], c, work[l], tst);
                        d[l + 1] = c;
                        work[l + 5] = tst;
                        rotateRight(2, z, l * 6 + 1, work, l + 1, l + 6);
                        e[l] = 0.0;
                        l += 2;
                        if (l + 1 > lend) {
                          exitg4 = 1;
                        }
                      } else if (jtot == 180) {
                        exitg4 = 1;
                      } else {
                        double g;
                        double p;
                        jtot++;
                        g = (d[l + 1] - d[l]) / (2.0 * e[l]);
                        c = rt_hypotd_snf(g, 1.0);
                        if (!(g >= 0.0)) {
                          c = -c;
                        }

                        g = (d[m - 1] - d[l]) + e[l] / (g + c);
                        tst = 1.0;
                        c = 1.0;
                        p = 0.0;
                        b_i = m - 1;
                        for (i = b_i; i >= l + 1; i--) {
                          double b;
                          double b_tmp;
                          b_tmp = e[i - 1];
                          b = c * b_tmp;
                          c = xzlartg(g, tst * b_tmp, s, r);
                          tst = s;
                          if (i != m - 1) {
                            e[i] = r;
                          }

                          g = d[i] - p;
                          r = (d[i - 1] - g) * s + 2.0 * c * b;
                          p = s * r;
                          d[i] = g + p;
                          g = c * r - b;
                          work[i - 1] = c;
                          work[i + 4] = -s;
                        }

                        rotateRight(m - l, z, l * 6 + 1, work, l + 1, l + 6);
                        d[l] -= p;
                        e[l] = g;
                      }
                    } while (exitg4 == 0);
                  } else {
                    int exitg3;
                    do {
                      exitg3 = 0;
                      if (l + 1 != lend) {
                        m = l + 1;
                        exitg2 = false;
                        while ((!exitg2) && (m > lend)) {
                          tst = std::abs(e[m - 2]);
                          if (tst * tst <= 4.9303806576313238E-32 * std::abs(d[m
                               - 1]) * std::abs(d[m - 2]) +
                              2.2250738585072014E-308) {
                            exitg2 = true;
                          } else {
                            m--;
                          }
                        }
                      } else {
                        m = lend;
                      }

                      if (m > lend) {
                        e[m - 2] = 0.0;
                      }

                      if (m == l + 1) {
                        l--;
                        if (l + 1 < lend) {
                          exitg3 = 1;
                        }
                      } else if (m == l) {
                        d[l - 1] = xdlaev2(d[l - 1], e[l - 1], d[l], c, work[m -
                                           1], tst);
                        d[l] = c;
                        work[m + 4] = tst;
                        b_rotateRight(2, z, (l - 1) * 6 + 1, work, m, m + 5);
                        e[l - 1] = 0.0;
                        l -= 2;
                        if (l + 1 < lend) {
                          exitg3 = 1;
                        }
                      } else if (jtot == 180) {
                        exitg3 = 1;
                      } else {
                        double g;
                        double p;
                        jtot++;
                        tst = e[l - 1];
                        g = (d[l - 1] - d[l]) / (2.0 * tst);
                        c = rt_hypotd_snf(g, 1.0);
                        if (!(g >= 0.0)) {
                          c = -c;
                        }

                        g = (d[m - 1] - d[l]) + tst / (g + c);
                        tst = 1.0;
                        c = 1.0;
                        p = 0.0;
                        for (i = m; i <= l; i++) {
                          double b;
                          double b_tmp;
                          b_tmp = e[i - 1];
                          b = c * b_tmp;
                          c = xzlartg(g, tst * b_tmp, s, r);
                          tst = s;
                          if (i != m) {
                            e[i - 2] = r;
                          }

                          g = d[i - 1] - p;
                          r = (d[i] - g) * s + 2.0 * c * b;
                          p = s * r;
                          d[i - 1] = g + p;
                          g = c * r - b;
                          work[i - 1] = c;
                          work[i + 4] = s;
                        }

                        b_rotateRight((l - m) + 2, z, (m - 1) * 6 + 1, work, m,
                                      m + 5);
                        d[l] -= p;
                        e[l - 1] = g;
                      }
                    } while (exitg3 == 0);
                  }

                  if (iscale == 1) {
                    b_i = lendsv - lsv;
                    xzlascl(2.2346346549904327E+153, anorm, b_i + 1, d, lsv);
                    b_xzlascl(2.2346346549904327E+153, anorm, b_i, e, lsv);
                  } else if (iscale == 2) {
                    b_i = lendsv - lsv;
                    xzlascl(3.02546243347603E-123, anorm, b_i + 1, d, lsv);
                    b_xzlascl(3.02546243347603E-123, anorm, b_i, e, lsv);
                  }

                  if (jtot >= 180) {
                    for (i = 0; i < 5; i++) {
                      if (e[i] != 0.0) {
                        info++;
                      }
                    }

                    exitg1 = 1;
                  }
                }
              }
            }
          }
        } while (exitg1 == 0);

        return info;
      }

      //
      // Arguments    : int ilo
      //                int ihi
      //                double A[36]
      //                const double tau[5]
      // Return Type  : void
      //
      static void xzunghr(int ilo, int ihi, double A[36], const double tau[5])
      {
        int a;
        int i;
        int ia;
        int ia0;
        int itau;
        int nh;
        nh = ihi - ilo;
        a = ilo + 1;
        for (int j{ihi}; j >= a; j--) {
          ia = (j - 1) * 6;
          i = static_cast<unsigned char>(j - 1);
          std::memset(&A[ia], 0, static_cast<unsigned int>((i + ia) - ia) *
                      sizeof(double));
          i = j + 1;
          for (int b_i{i}; b_i <= ihi; b_i++) {
            itau = ia + b_i;
            A[itau - 1] = A[itau - 7];
          }

          i = ihi + 1;
          if (i <= 6) {
            std::memset(&A[(i + ia) + -1], 0, static_cast<unsigned int>(((ia - i)
              - ia) + 7) * sizeof(double));
          }
        }

        i = static_cast<unsigned char>(ilo);
        for (int j{0}; j < i; j++) {
          ia = j * 6;
          for (int b_i{0}; b_i < 6; b_i++) {
            A[ia + b_i] = 0.0;
          }

          A[ia + j] = 1.0;
        }

        i = ihi + 1;
        for (int j{i}; j < 7; j++) {
          ia = (j - 1) * 6;
          for (int b_i{0}; b_i < 6; b_i++) {
            A[ia + b_i] = 0.0;
          }

          A[(ia + j) - 1] = 1.0;
        }

        ia0 = ilo + ilo * 6;
        if (nh >= 1) {
          double work[6];
          i = nh - 1;
          for (int j{nh}; j <= i; j++) {
            ia = ia0 + j * 6;
            std::memset(&A[ia], 0, static_cast<unsigned int>(((i + ia) - ia) + 1)
                        * sizeof(double));
            A[ia + j] = 1.0;
          }

          itau = (ilo + nh) - 2;
          for (int b_i{0}; b_i < 6; b_i++) {
            work[b_i] = 0.0;
          }

          for (int b_i{nh}; b_i >= 1; b_i--) {
            ia = (ia0 + b_i) + (b_i - 1) * 6;
            if (b_i < nh) {
              A[ia - 1] = 1.0;
              i = nh - b_i;
              xzlarf(i + 1, i, ia, tau[itau], A, ia + 6, work);
              a = ia + 1;
              i = (ia + nh) - b_i;
              for (int j{a}; j <= i; j++) {
                A[j - 1] *= -tau[itau];
              }
            }

            A[ia - 1] = 1.0 - tau[itau];
            i = static_cast<unsigned char>(b_i - 1);
            for (int j{0}; j < i; j++) {
              A[(ia - j) - 2] = 0.0;
            }

            itau--;
          }
        }
      }

      //
      // Arguments    : array<unsigned int, 1U> &x
      // Return Type  : void
      //
    }

    static void sort(array<unsigned int, 1U> &x)
    {
      array<int, 1U> iidx;
      array<int, 1U> iwork;
      array<unsigned int, 1U> vwork;
      array<unsigned int, 1U> xwork;
      int dim;
      int i;
      int vlen;
      int vstride;
      dim = 0;
      if (x.size(0) != 1) {
        dim = -1;
      }

      if (dim + 2 <= 1) {
        i = x.size(0);
      } else {
        i = 1;
      }

      vlen = i - 1;
      vwork.set_size(i);
      vstride = 1;
      for (int k{0}; k <= dim; k++) {
        vstride *= x.size(0);
      }

      for (int b_i{0}; b_i < 1; b_i++) {
        for (int j{0}; j < vstride; j++) {
          for (int k{0}; k <= vlen; k++) {
            vwork[k] = x[j + k * vstride];
          }

          dim = vwork.size(0);
          iidx.set_size(vwork.size(0));
          for (i = 0; i < dim; i++) {
            iidx[i] = 0;
          }

          dim = vwork.size(0);
          if (vwork.size(0) != 0) {
            int idx4[4];
            unsigned int x4[4];
            int c_i;
            int i1;
            int i2;
            int i3;
            int i4;
            int offset;
            x4[0] = 0U;
            idx4[0] = 0;
            x4[1] = 0U;
            idx4[1] = 0;
            x4[2] = 0U;
            idx4[2] = 0;
            x4[3] = 0U;
            idx4[3] = 0;
            iwork.set_size(vwork.size(0));
            xwork.set_size(vwork.size(0));
            for (i = 0; i < dim; i++) {
              iwork[i] = 0;
              xwork[i] = 0U;
            }

            dim = vwork.size(0) >> 2;
            for (int b_j{0}; b_j < dim; b_j++) {
              unsigned int b_x4_tmp;
              unsigned int c_x4_tmp;
              unsigned int x4_tmp;
              c_i = b_j << 2;
              idx4[0] = c_i + 1;
              idx4[1] = c_i + 2;
              idx4[2] = c_i + 3;
              idx4[3] = c_i + 4;
              x4[0] = vwork[c_i];
              x4_tmp = vwork[c_i + 1];
              x4[1] = x4_tmp;
              b_x4_tmp = vwork[c_i + 2];
              x4[2] = b_x4_tmp;
              c_x4_tmp = vwork[c_i + 3];
              x4[3] = c_x4_tmp;
              if (vwork[c_i] <= x4_tmp) {
                i1 = 1;
                i2 = 2;
              } else {
                i1 = 2;
                i2 = 1;
              }

              if (b_x4_tmp <= c_x4_tmp) {
                i3 = 3;
                i4 = 4;
              } else {
                i3 = 4;
                i4 = 3;
              }

              x4_tmp = x4[i3 - 1];
              b_x4_tmp = x4[i1 - 1];
              if (b_x4_tmp <= x4_tmp) {
                b_x4_tmp = x4[i2 - 1];
                if (b_x4_tmp <= x4_tmp) {
                  i = i1;
                  offset = i2;
                  i1 = i3;
                  i2 = i4;
                } else if (b_x4_tmp <= x4[i4 - 1]) {
                  i = i1;
                  offset = i3;
                  i1 = i2;
                  i2 = i4;
                } else {
                  i = i1;
                  offset = i3;
                  i1 = i4;
                }
              } else {
                x4_tmp = x4[i4 - 1];
                if (b_x4_tmp <= x4_tmp) {
                  if (x4[i2 - 1] <= x4_tmp) {
                    i = i3;
                    offset = i1;
                    i1 = i2;
                    i2 = i4;
                  } else {
                    i = i3;
                    offset = i1;
                    i1 = i4;
                  }
                } else {
                  i = i3;
                  offset = i4;
                }
              }

              iidx[c_i] = idx4[i - 1];
              iidx[c_i + 1] = idx4[offset - 1];
              iidx[c_i + 2] = idx4[i1 - 1];
              iidx[c_i + 3] = idx4[i2 - 1];
              vwork[c_i] = x4[i - 1];
              vwork[c_i + 1] = x4[offset - 1];
              vwork[c_i + 2] = x4[i1 - 1];
              vwork[c_i + 3] = x4[i2 - 1];
            }

            c_i = dim << 2;
            i1 = vwork.size(0) - c_i;
            if (i1 > 0) {
              signed char perm[4];
              for (int k{0}; k < i1; k++) {
                dim = c_i + k;
                idx4[k] = dim + 1;
                x4[k] = vwork[dim];
              }

              perm[1] = 0;
              perm[2] = 0;
              perm[3] = 0;
              if (i1 == 1) {
                perm[0] = 1;
              } else if (i1 == 2) {
                if (x4[0] <= x4[1]) {
                  perm[0] = 1;
                  perm[1] = 2;
                } else {
                  perm[0] = 2;
                  perm[1] = 1;
                }
              } else if (x4[0] <= x4[1]) {
                if (x4[1] <= x4[2]) {
                  perm[0] = 1;
                  perm[1] = 2;
                  perm[2] = 3;
                } else if (x4[0] <= x4[2]) {
                  perm[0] = 1;
                  perm[1] = 3;
                  perm[2] = 2;
                } else {
                  perm[0] = 3;
                  perm[1] = 1;
                  perm[2] = 2;
                }
              } else if (x4[0] <= x4[2]) {
                perm[0] = 2;
                perm[1] = 1;
                perm[2] = 3;
              } else if (x4[1] <= x4[2]) {
                perm[0] = 2;
                perm[1] = 3;
                perm[2] = 1;
              } else {
                perm[0] = 3;
                perm[1] = 2;
                perm[2] = 1;
              }

              for (int k{0}; k < i1; k++) {
                i2 = c_i + k;
                i = perm[k];
                iidx[i2] = idx4[i - 1];
                vwork[i2] = x4[i - 1];
              }
            }

            dim = 2;
            if (vwork.size(0) > 1) {
              if (vwork.size(0) >= 256) {
                i4 = vwork.size(0) >> 8;
                for (int b{0}; b < i4; b++) {
                  int b_iwork[256];
                  unsigned int b_xwork[256];
                  offset = (b << 8) - 1;
                  for (int b_b{0}; b_b < 6; b_b++) {
                    int bLen;
                    int bLen2;
                    bLen = 1 << (b_b + 2);
                    bLen2 = bLen << 1;
                    i = 256 >> (b_b + 3);
                    for (int k{0}; k < i; k++) {
                      i1 = (offset + k * bLen2) + 1;
                      for (int b_j{0}; b_j < bLen2; b_j++) {
                        dim = i1 + b_j;
                        b_iwork[b_j] = iidx[dim];
                        b_xwork[b_j] = vwork[dim];
                      }

                      i3 = 0;
                      c_i = bLen;
                      dim = i1 - 1;
                      int exitg1;
                      do {
                        exitg1 = 0;
                        dim++;
                        if (b_xwork[i3] <= b_xwork[c_i]) {
                          iidx[dim] = b_iwork[i3];
                          vwork[dim] = b_xwork[i3];
                          if (i3 + 1 < bLen) {
                            i3++;
                          } else {
                            exitg1 = 1;
                          }
                        } else {
                          iidx[dim] = b_iwork[c_i];
                          vwork[dim] = b_xwork[c_i];
                          if (c_i + 1 < bLen2) {
                            c_i++;
                          } else {
                            dim -= i3;
                            for (int b_j{i3 + 1}; b_j <= bLen; b_j++) {
                              i2 = dim + b_j;
                              iidx[i2] = b_iwork[b_j - 1];
                              vwork[i2] = b_xwork[b_j - 1];
                            }

                            exitg1 = 1;
                          }
                        }
                      } while (exitg1 == 0);
                    }
                  }
                }

                dim = i4 << 8;
                c_i = vwork.size(0) - dim;
                if (c_i > 0) {
                  merge_block(iidx, vwork, dim, c_i, 2, iwork, xwork);
                }

                dim = 8;
              }

              merge_block(iidx, vwork, 0, vwork.size(0), dim, iwork, xwork);
            }
          }

          for (int k{0}; k <= vlen; k++) {
            x[j + k * vstride] = vwork[k];
          }
        }
      }
    }

    //
    // Arguments    : array<double, 2U> &x
    //                array<int, 2U> &idx
    // Return Type  : void
    //
    static void sort(array<double, 2U> &x, array<int, 2U> &idx)
    {
      array<double, 1U> xwork;
      array<int, 1U> iwork;
      int i;
      int ib;
      idx.set_size(1, x.size(1));
      ib = x.size(1);
      for (i = 0; i < ib; i++) {
        idx[i] = 0;
      }

      if (x.size(1) != 0) {
        double x4[4];
        int idx4[4];
        int bLen;
        int bLen2;
        int i1;
        int i2;
        int i3;
        int i4;
        int idx_tmp;
        int n;
        int wOffset_tmp;
        n = x.size(1);
        x4[0] = 0.0;
        idx4[0] = 0;
        x4[1] = 0.0;
        idx4[1] = 0;
        x4[2] = 0.0;
        idx4[2] = 0;
        x4[3] = 0.0;
        idx4[3] = 0;
        iwork.set_size(x.size(1));
        ib = x.size(1);
        xwork.set_size(x.size(1));
        for (i = 0; i < ib; i++) {
          iwork[i] = 0;
          xwork[i] = 0.0;
        }

        bLen2 = 0;
        ib = 0;
        for (int k{0}; k < n; k++) {
          if (std::isnan(x[k])) {
            idx_tmp = (n - bLen2) - 1;
            idx[idx_tmp] = k + 1;
            xwork[idx_tmp] = x[k];
            bLen2++;
          } else {
            ib++;
            idx4[ib - 1] = k + 1;
            x4[ib - 1] = x[k];
            if (ib == 4) {
              double d;
              double d1;
              ib = k - bLen2;
              if (x4[0] <= x4[1]) {
                i1 = 1;
                i2 = 2;
              } else {
                i1 = 2;
                i2 = 1;
              }

              if (x4[2] <= x4[3]) {
                i3 = 3;
                i4 = 4;
              } else {
                i3 = 4;
                i4 = 3;
              }

              d = x4[i3 - 1];
              d1 = x4[i1 - 1];
              if (d1 <= d) {
                d1 = x4[i2 - 1];
                if (d1 <= d) {
                  i = i1;
                  bLen = i2;
                  i1 = i3;
                  i2 = i4;
                } else if (d1 <= x4[i4 - 1]) {
                  i = i1;
                  bLen = i3;
                  i1 = i2;
                  i2 = i4;
                } else {
                  i = i1;
                  bLen = i3;
                  i1 = i4;
                }
              } else {
                d = x4[i4 - 1];
                if (d1 <= d) {
                  if (x4[i2 - 1] <= d) {
                    i = i3;
                    bLen = i1;
                    i1 = i2;
                    i2 = i4;
                  } else {
                    i = i3;
                    bLen = i1;
                    i1 = i4;
                  }
                } else {
                  i = i3;
                  bLen = i4;
                }
              }

              idx[ib - 3] = idx4[i - 1];
              idx[ib - 2] = idx4[bLen - 1];
              idx[ib - 1] = idx4[i1 - 1];
              idx[ib] = idx4[i2 - 1];
              x[ib - 3] = x4[i - 1];
              x[ib - 2] = x4[bLen - 1];
              x[ib - 1] = x4[i1 - 1];
              x[ib] = x4[i2 - 1];
              ib = 0;
            }
          }
        }

        wOffset_tmp = x.size(1) - bLen2;
        if (ib > 0) {
          signed char perm[4];
          perm[1] = 0;
          perm[2] = 0;
          perm[3] = 0;
          if (ib == 1) {
            perm[0] = 1;
          } else if (ib == 2) {
            if (x4[0] <= x4[1]) {
              perm[0] = 1;
              perm[1] = 2;
            } else {
              perm[0] = 2;
              perm[1] = 1;
            }
          } else if (x4[0] <= x4[1]) {
            if (x4[1] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 2;
              perm[2] = 3;
            } else if (x4[0] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 3;
              perm[2] = 2;
            } else {
              perm[0] = 3;
              perm[1] = 1;
              perm[2] = 2;
            }
          } else if (x4[0] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 1;
            perm[2] = 3;
          } else if (x4[1] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 3;
            perm[2] = 1;
          } else {
            perm[0] = 3;
            perm[1] = 2;
            perm[2] = 1;
          }

          i = static_cast<unsigned char>(ib);
          for (int k{0}; k < i; k++) {
            idx_tmp = (wOffset_tmp - ib) + k;
            bLen = perm[k];
            idx[idx_tmp] = idx4[bLen - 1];
            x[idx_tmp] = x4[bLen - 1];
          }
        }

        i1 = bLen2 >> 1;
        for (int k{0}; k < i1; k++) {
          ib = wOffset_tmp + k;
          i2 = idx[ib];
          idx_tmp = (n - k) - 1;
          idx[ib] = idx[idx_tmp];
          idx[idx_tmp] = i2;
          x[ib] = xwork[idx_tmp];
          x[idx_tmp] = xwork[ib];
        }

        if ((bLen2 & 1) != 0) {
          i = wOffset_tmp + i1;
          x[i] = xwork[i];
        }

        ib = 2;
        if (wOffset_tmp > 1) {
          if (x.size(1) >= 256) {
            n = wOffset_tmp >> 8;
            if (n > 0) {
              for (int b{0}; b < n; b++) {
                double b_xwork[256];
                int b_iwork[256];
                i4 = (b << 8) - 1;
                for (int b_b{0}; b_b < 6; b_b++) {
                  bLen = 1 << (b_b + 2);
                  bLen2 = bLen << 1;
                  i = 256 >> (b_b + 3);
                  for (int k{0}; k < i; k++) {
                    i2 = (i4 + k * bLen2) + 1;
                    for (i1 = 0; i1 < bLen2; i1++) {
                      ib = i2 + i1;
                      b_iwork[i1] = idx[ib];
                      b_xwork[i1] = x[ib];
                    }

                    i3 = 0;
                    i1 = bLen;
                    ib = i2 - 1;
                    int exitg1;
                    do {
                      exitg1 = 0;
                      ib++;
                      if (b_xwork[i3] <= b_xwork[i1]) {
                        idx[ib] = b_iwork[i3];
                        x[ib] = b_xwork[i3];
                        if (i3 + 1 < bLen) {
                          i3++;
                        } else {
                          exitg1 = 1;
                        }
                      } else {
                        idx[ib] = b_iwork[i1];
                        x[ib] = b_xwork[i1];
                        if (i1 + 1 < bLen2) {
                          i1++;
                        } else {
                          ib -= i3;
                          for (i1 = i3 + 1; i1 <= bLen; i1++) {
                            idx_tmp = ib + i1;
                            idx[idx_tmp] = b_iwork[i1 - 1];
                            x[idx_tmp] = b_xwork[i1 - 1];
                          }

                          exitg1 = 1;
                        }
                      }
                    } while (exitg1 == 0);
                  }
                }
              }

              ib = n << 8;
              i1 = wOffset_tmp - ib;
              if (i1 > 0) {
                merge_block(idx, x, ib, i1, 2, iwork, xwork);
              }

              ib = 8;
            }
          }

          merge_block(idx, x, 0, wOffset_tmp, ib, iwork, xwork);
        }
      }
    }

    //
    // Arguments    : const double A[36]
    //                double Y[36]
    // Return Type  : void
    //
  }

  static void mldivide(const double A[36], double Y[36])
  {
    double b_A[36];
    int ipiv[6];
    int Y_tmp;
    int i;
    int jBcol;
    int kAcol;
    int temp;
    for (i = 0; i < 36; i++) {
      Y[i] = iv[i];
      b_A[i] = A[i];
    }

    internal::reflapack::xzgetrf(b_A, ipiv);
    for (int b_i{0}; b_i < 5; b_i++) {
      i = ipiv[b_i];
      if (i != b_i + 1) {
        for (int j{0}; j < 6; j++) {
          jBcol = b_i + 6 * j;
          temp = static_cast<int>(Y[jBcol]);
          Y_tmp = (i + 6 * j) - 1;
          Y[jBcol] = Y[Y_tmp];
          Y[Y_tmp] = temp;
        }
      }
    }

    for (int j{0}; j < 6; j++) {
      jBcol = 6 * j;
      for (temp = 0; temp < 6; temp++) {
        kAcol = 6 * temp;
        i = temp + jBcol;
        if (Y[i] != 0.0) {
          int i1;
          i1 = temp + 2;
          for (int b_i{i1}; b_i < 7; b_i++) {
            Y_tmp = (b_i + jBcol) - 1;
            Y[Y_tmp] -= Y[i] * b_A[(b_i + kAcol) - 1];
          }
        }
      }
    }

    for (int j{0}; j < 6; j++) {
      jBcol = 6 * j;
      for (temp = 5; temp >= 0; temp--) {
        double d;
        kAcol = 6 * temp;
        i = temp + jBcol;
        d = Y[i];
        if (d != 0.0) {
          Y[i] = d / b_A[temp + kAcol];
          for (int b_i{0}; b_i < temp; b_i++) {
            Y_tmp = b_i + jBcol;
            Y[Y_tmp] -= Y[i] * b_A[b_i + kAcol];
          }
        }
      }
    }
  }

  //
  // Arguments    : const array<double, 2U> &A
  //                array<int, 1U> &m1
  //                array<int, 1U> &m2
  // Return Type  : void
  //
  static void perfectMatching(const array<double, 2U> &A, array<int, 1U> &m1,
    array<int, 1U> &m2)
  {
    matlab::internal::coder::minPriorityQueue b_queue;
    matlab::internal::coder::minPriorityQueue queue;
    array<double, 1U> a__2;
    array<double, 1U> distancesR;
    array<double, 1U> ex;
    array<double, 1U> minIndices;
    array<double, 1U> pairWeightR;
    array<int, 1U> idx;
    array<unsigned char, 1U> colorsR;
    int c;
    int i;
    int n_tmp;
    n_tmp = A.size(0) - 1;
    m1.set_size(A.size(0));
    c = A.size(0);
    m2.set_size(A.size(0));
    a__2.set_size(A.size(0));
    for (i = 0; i < c; i++) {
      m1[i] = 0;
      m2[i] = 0;
      a__2[i] = rtInf;
    }

    if (n_tmp + 1 != 0) {
      double edge_weight_shifted;
      int b_i;
      int n;
      boolean_T guard1;
      boolean_T p;
      n = A.size(1);
      ex.set_size(A.size(0));
      idx.set_size(A.size(0));
      c = A.size(0);
      for (i = 0; i < c; i++) {
        idx[i] = 1;
      }

      if (A.size(0) >= 1) {
        for (b_i = 0; b_i <= n_tmp; b_i++) {
          ex[b_i] = A[b_i];
        }

        for (c = 2; c <= n; c++) {
          for (b_i = 0; b_i <= n_tmp; b_i++) {
            edge_weight_shifted = A[b_i + A.size(0) * (c - 1)];
            if (std::isnan(edge_weight_shifted)) {
              p = false;
            } else if (std::isnan(ex[b_i])) {
              p = true;
            } else {
              p = (ex[b_i] > edge_weight_shifted);
            }

            if (p) {
              ex[b_i] = edge_weight_shifted;
              idx[b_i] = c;
            }
          }
        }
      }

      minIndices.set_size(idx.size(0));
      c = idx.size(0);
      p = true;
      for (n = 0; n < c; n++) {
        minIndices[n] = idx[n];
        if ((!p) || (std::isinf(ex[n]) || std::isnan(ex[n]))) {
          p = false;
        }
      }

      guard1 = false;
      if (!p) {
        guard1 = true;
      } else {
        int colStart;
        boolean_T exitg1;
        pairWeightR.set_size(n_tmp + 1);
        for (n = 0; n <= n_tmp; n++) {
          pairWeightR[n] = 0.0;
          i = static_cast<int>(minIndices[n]);
          if (m1[i - 1] == 0) {
            m2[n] = i;
            m1[i - 1] = n + 1;
            pairWeightR[n] = ex[n];
          }

          for (c = 0; c <= n_tmp; c++) {
            edge_weight_shifted = A[c + A.size(0) * n] - ex[c];
            if (edge_weight_shifted < a__2[n]) {
              a__2[n] = edge_weight_shifted;
            }
          }
        }

        queue.heap.set_size(n_tmp + 1);
        queue.indexToHeap.set_size(n_tmp + 1);
        queue.len = 0;
        colStart = 0;
        exitg1 = false;
        while ((!exitg1) && (colStart <= n_tmp)) {
          if (m1[colStart] != 0) {
            colStart++;
          } else {
            double last_weight_sap;
            double lsap;
            int clast;
            int exitg2;
            int rlast;
            boolean_T guard2;
            b_queue = queue;
            minIndices.set_size(n_tmp + 1);
            idx.set_size(n_tmp + 1);
            distancesR.set_size(n_tmp + 1);
            colorsR.set_size(n_tmp + 1);
            for (i = 0; i <= n_tmp; i++) {
              minIndices[i] = 0.0;
              idx[i] = 0;
              distancesR[i] = rtInf;
              colorsR[i] = 0U;
            }

            idx[colStart] = 0;
            b_queue.len = 0;
            edge_weight_shifted = 0.0;
            lsap = rtInf;
            rlast = -1;
            clast = -1;
            last_weight_sap = rtInf;
            c = colStart;
            guard2 = false;
            int exitg3;
            do {
              double edge_weight;
              exitg3 = 0;
              n = 0;
              do {
                exitg2 = 0;
                if (n <= n_tmp) {
                  if (colorsR[n] == 2) {
                    n++;
                  } else {
                    double dnew;
                    edge_weight = A[n + A.size(0) * c];
                    dnew = edge_weight_shifted + ((edge_weight - ex[n]) - a__2[c]);
                    if (dnew < lsap) {
                      if (m2[n] == 0) {
                        lsap = dnew;
                        rlast = n;
                        clast = c;
                        last_weight_sap = edge_weight;
                      } else if (dnew < distancesR[n]) {
                        distancesR[n] = dnew;
                        idx[m2[n] - 1] = c + 1;
                        minIndices[n] = edge_weight;
                        if (colorsR[n] == 0) {
                          b_queue.len++;
                          b_queue.heap[b_queue.len - 1] = n + 1;
                          b_queue.indexToHeap[n] = b_queue.len;
                          b_queue.percUp(b_queue.len, distancesR);
                          colorsR[n] = 1U;
                        } else {
                          b_queue.percUp(b_queue.indexToHeap[n], distancesR);
                        }
                      }

                      n++;
                    } else if (((dnew >= -1.7976931348623157E+308) && (dnew <=
                                 1.7976931348623157E+308)) || (!(edge_weight >=
                                 -1.7976931348623157E+308)) || (!(edge_weight <=
                      1.7976931348623157E+308))) {
                      n++;
                    } else {
                      p = false;
                      exitg2 = 2;
                    }
                  }
                } else {
                  exitg2 = 1;
                }
              } while (exitg2 == 0);

              if (exitg2 == 1) {
                if (b_queue.len == 0) {
                  guard2 = true;
                  exitg3 = 1;
                } else {
                  c = b_queue.heap[0] - 1;
                  i = b_queue.len - 1;
                  b_queue.heap[0] = b_queue.heap[b_queue.len - 1];
                  b_queue.indexToHeap[b_queue.heap[0] - 1] = 1;
                  b_queue.len--;
                  b_i = 0;
                  int exitg4;
                  int ichild;
                  do {
                    exitg4 = 0;
                    ichild = (b_i + 1) << 1;
                    if (ichild <= i) {
                      if (ichild + 1 > i) {
                        ichild--;
                      } else {
                        n = b_queue.heap[ichild - 1];
                        edge_weight_shifted = distancesR[n - 1];
                        edge_weight = distancesR[b_queue.heap[ichild] - 1];
                        if ((edge_weight_shifted < edge_weight) ||
                            ((edge_weight_shifted == edge_weight) && (n <=
                              b_queue.heap[ichild]))) {
                          ichild--;
                        }
                      }

                      edge_weight_shifted = distancesR[b_queue.heap[b_i] - 1];
                      edge_weight = distancesR[b_queue.heap[ichild] - 1];
                      if ((edge_weight_shifted < edge_weight) ||
                          ((edge_weight_shifted == edge_weight) &&
                           (b_queue.heap[b_i] <= b_queue.heap[ichild]))) {
                        exitg4 = 1;
                      } else {
                        n = b_queue.heap[b_i];
                        b_queue.heap[b_i] = b_queue.heap[ichild];
                        b_queue.heap[ichild] = n;
                        n = b_queue.indexToHeap[b_queue.heap[b_i] - 1];
                        b_queue.indexToHeap[b_queue.heap[b_i] - 1] =
                          b_queue.indexToHeap[b_queue.heap[ichild] - 1];
                        b_queue.indexToHeap[b_queue.heap[ichild] - 1] = n;
                        b_i = ichild;
                      }
                    } else {
                      exitg4 = 1;
                    }
                  } while (exitg4 == 0);

                  edge_weight_shifted = distancesR[c];
                  if (lsap <= distancesR[c]) {
                    guard2 = true;
                    exitg3 = 1;
                  } else {
                    colorsR[c] = 2U;
                    c = m2[c] - 1;
                    guard2 = false;
                  }
                }
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            if (guard2) {
              p = (lsap < rtInf);
              if (p) {
                c = rlast + 1;
                do {
                  exitg2 = 0;
                  n = m1[clast];
                  m2[c - 1] = clast + 1;
                  m1[clast] = c;
                  pairWeightR[c - 1] = last_weight_sap;
                  if (idx[clast] == 0) {
                    exitg2 = 1;
                  } else {
                    c = n;
                    clast = idx[clast] - 1;
                    last_weight_sap = minIndices[n - 1];
                  }
                } while (exitg2 == 0);

                for (n = 0; n <= n_tmp; n++) {
                  if (colorsR[n] == 2) {
                    ex[n] = (ex[n] - lsap) + distancesR[n];
                  }
                }

                for (c = 0; c <= n_tmp; c++) {
                  if (m1[c] != 0) {
                    a__2[c] = pairWeightR[m1[c] - 1] - ex[m1[c] - 1];
                  }
                }
              }
            }

            if (!p) {
              guard1 = true;
              exitg1 = true;
            } else {
              colStart++;
            }
          }
        }
      }

      if (guard1) {
        m1.set_size(0);
        m2.set_size(0);
      }
    }
  }

  //
  // Arguments    : const array<double, 2U> &a
  //                array<double, 2U> &b
  // Return Type  : void
  //
  static void unique_vector(const array<double, 2U> &a, array<double, 2U> &b)
  {
    array<int, 2U> idx;
    array<int, 1U> iwork;
    double x;
    int b_i;
    int i;
    int i2;
    int j;
    int k;
    int n;
    int na;
    int nb;
    int pEnd;
    int qEnd;
    boolean_T exitg1;
    na = a.size(1);
    n = a.size(1) + 1;
    idx.set_size(1, a.size(1));
    i = a.size(1);
    for (b_i = 0; b_i < i; b_i++) {
      idx[b_i] = 0;
    }

    if (a.size(1) != 0) {
      iwork.set_size(a.size(1));
      b_i = a.size(1) - 1;
      for (k = 1; k <= b_i; k += 2) {
        x = a[k];
        if ((a[k - 1] <= x) || std::isnan(x)) {
          idx[k - 1] = k;
          idx[k] = k + 1;
        } else {
          idx[k - 1] = k + 1;
          idx[k] = k;
        }
      }

      if ((a.size(1) & 1) != 0) {
        idx[a.size(1) - 1] = a.size(1);
      }

      i = 2;
      while (i < n - 1) {
        i2 = i << 1;
        j = 1;
        for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
          int kEnd;
          int q;
          nb = j;
          q = pEnd - 1;
          qEnd = j + i2;
          if (qEnd > n) {
            qEnd = n;
          }

          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            x = a[idx[q] - 1];
            b_i = idx[nb - 1];
            if ((a[b_i - 1] <= x) || std::isnan(x)) {
              iwork[k] = b_i;
              nb++;
              if (nb == pEnd) {
                while (q + 1 < qEnd) {
                  k++;
                  iwork[k] = idx[q];
                  q++;
                }
              }
            } else {
              iwork[k] = idx[q];
              q++;
              if (q + 1 == qEnd) {
                while (nb < pEnd) {
                  k++;
                  iwork[k] = idx[nb - 1];
                  nb++;
                }
              }
            }

            k++;
          }

          for (k = 0; k < kEnd; k++) {
            idx[(j + k) - 1] = iwork[k];
          }

          j = qEnd;
        }

        i = i2;
      }
    }

    b.set_size(1, a.size(1));
    for (k = 0; k < na; k++) {
      b[k] = a[idx[k] - 1];
    }

    k = 0;
    while ((k + 1 <= na) && std::isinf(b[k]) && (b[k] < 0.0)) {
      k++;
    }

    i2 = k;
    k = a.size(1);
    while ((k >= 1) && std::isnan(b[k - 1])) {
      k--;
    }

    pEnd = a.size(1) - k;
    exitg1 = false;
    while ((!exitg1) && (k >= 1)) {
      x = b[k - 1];
      if (std::isinf(x) && (x > 0.0)) {
        k--;
      } else {
        exitg1 = true;
      }
    }

    i = (a.size(1) - k) - pEnd;
    nb = -1;
    if (i2 > 0) {
      nb = 0;
    }

    while (i2 + 1 <= k) {
      x = b[i2];
      do {
        i2++;
      } while (!((i2 + 1 > k) || (b[i2] != x)));

      nb++;
      b[nb] = x;
    }

    if (i > 0) {
      nb++;
      b[nb] = b[k];
    }

    i2 = k + i;
    for (j = 0; j < pEnd; j++) {
      b[(nb + j) + 1] = b[i2 + j];
    }

    if (pEnd - 1 >= 0) {
      nb += pEnd;
    }

    if (nb + 1 < 1) {
      b_i = 0;
    } else {
      b_i = nb + 1;
    }

    b.set_size(b.size(0), b_i);
  }

  //
  // Arguments    : const array<unsigned int, 1U> &a
  //                array<unsigned int, 1U> &b
  // Return Type  : void
  //
  static void unique_vector(const array<unsigned int, 1U> &a, array<unsigned int,
    1U> &b)
  {
    array<int, 1U> idx;
    array<int, 1U> iwork;
    int b_i;
    int i;
    int k;
    int n;
    int na;
    int qEnd;
    na = a.size(0);
    n = a.size(0) + 1;
    idx.set_size(a.size(0));
    i = a.size(0);
    for (b_i = 0; b_i < i; b_i++) {
      idx[b_i] = 0;
    }

    if (a.size(0) != 0) {
      iwork.set_size(a.size(0));
      b_i = a.size(0) - 1;
      for (k = 1; k <= b_i; k += 2) {
        if (a[k - 1] <= a[k]) {
          idx[k - 1] = k;
          idx[k] = k + 1;
        } else {
          idx[k - 1] = k + 1;
          idx[k] = k;
        }
      }

      if ((a.size(0) & 1) != 0) {
        idx[a.size(0) - 1] = a.size(0);
      }

      i = 2;
      while (i < n - 1) {
        int i2;
        int j;
        i2 = i << 1;
        j = 1;
        for (int pEnd{i + 1}; pEnd < n; pEnd = qEnd + i) {
          int kEnd;
          int p;
          int q;
          p = j;
          q = pEnd;
          qEnd = j + i2;
          if (qEnd > n) {
            qEnd = n;
          }

          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            int i1;
            b_i = idx[p - 1];
            i1 = idx[q - 1];
            if (a[b_i - 1] <= a[i1 - 1]) {
              iwork[k] = b_i;
              p++;
              if (p == pEnd) {
                while (q < qEnd) {
                  k++;
                  iwork[k] = idx[q - 1];
                  q++;
                }
              }
            } else {
              iwork[k] = i1;
              q++;
              if (q == qEnd) {
                while (p < pEnd) {
                  k++;
                  iwork[k] = idx[p - 1];
                  p++;
                }
              }
            }

            k++;
          }

          for (k = 0; k < kEnd; k++) {
            idx[(j + k) - 1] = iwork[k];
          }

          j = qEnd;
        }

        i = i2;
      }
    }

    b.set_size(a.size(0));
    for (k = 0; k < na; k++) {
      b[k] = a[idx[k] - 1];
    }

    i = 0;
    k = 1;
    while (k <= na) {
      unsigned int x;
      x = b[k - 1];
      do {
        k++;
      } while (!((k > na) || (b[k - 1] != x)));

      i++;
      b[i - 1] = x;
    }

    if (i < 1) {
      i = 0;
    }

    b.set_size(i);
  }

  //
  // Arguments    : int numerator
  // Return Type  : int
  //
}

static int div_nde_s32_floor(int numerator)
{
  int i;
  if ((numerator < 0) && (numerator % 6 != 0)) {
    i = -1;
  } else {
    i = 0;
  }

  return numerator / 6 + i;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (std::isnan(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : void
// Return Type  : trackFuser
//
namespace coder
{
  trackFuser::trackFuser()
  {
    matlabCodegenIsDeleted = true;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  trackFuser::~trackFuser()
  {
    matlabCodegenDestructor();
  }

  //
  // function [confirmedTracks,tentativeTracks,allTracks,analysisInformation] = EDGE_fusion_function_231017_2135(hub_data_Object)
  //
  // UNTITLED9 이 함수의 요약 설명 위치
  //    자세한 설명 위치
  //
  // Arguments    : const struct0_T *hub_data_Object
  //                struct3_T confirmedTracks_data[]
  //                int confirmedTracks_size[1]
  //                struct3_T tentativeTracks_data[]
  //                int tentativeTracks_size[1]
  //                coder::array<struct3_T, 1U> &allTracks
  //                struct4_T *analysisInformation
  // Return Type  : void
  //
}

void EDGE_fusion_function_231017_2135(const struct0_T *hub_data_Object,
  struct3_T confirmedTracks_data[], int confirmedTracks_size[1], struct3_T
  tentativeTracks_data[], int tentativeTracks_size[1], coder::array<struct3_T,
  1U> &allTracks, struct4_T *analysisInformation)
{
  static const struct_T r{ 1U,         // TrackID
    0U,                                // BranchID
    1U,                                // SourceIndex
    0.0,                               // UpdateTime
    1U,                                // Age

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // State

    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },  // StateCovariance
    0.0,                               // ObjectClassID
    1.0,                               // ObjectClassProbabilities

    { 'H', 'i', 's', 't', 'o', 'r', 'y' },// TrackLogic
    true,                              // TrackLogicState
    true,                              // IsConfirmed
    false,                             // IsCoasted
    true                               // IsSelfReported
  };

  static const signed char stateCov[36]{ 5, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0,
    0, 5, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 5 };

  static const signed char b_iv[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const char logicType[7]{ 'H', 'i', 's', 't', 'o', 'r', 'y' };

  coder::fuserSourceConfiguration lobj_1[5];
  coder::fuserSourceConfiguration *sourceconfig_ego[5];
  coder::fuserSourceConfiguration *b_obj;
  coder::fuserSourceConfiguration *obj;
  coder::trackFuser fuser;
  coder::array<struct_T, 2U> track_ego;
  struct_T expl_temp;
  double ids[20];
  double d;
  int b_track_ego;
  boolean_T flag;
  fuser.matlabCodegenIsDeleted = true;

  // 'EDGE_fusion_function_231017_2135:4' maxobs=20;
  //  차량 1대당 최대 인식 장애물 갯수
  // 'EDGE_fusion_function_231017_2135:5' numveh=5;
  //  차량 댓수
  //  Preallocation
  // 'EDGE_fusion_function_231017_2135:7' track_ego=toStruct(objectTrack());
  track_ego.set_size(1, 1);
  track_ego[0] = r;

  // 'EDGE_fusion_function_231017_2135:8' temp_track=toStruct(objectTrack());
  // 'EDGE_fusion_function_231017_2135:11' state=struct('obs_state',[0;0;0;0;0;0],'obs_time',0,'obs_class',0);
  // 'EDGE_fusion_function_231017_2135:12' target_vehicle=struct('state',struct('obs_state',[0;0;0;0;0;0],'obs_time',0,'obs_class',0));
  // 'EDGE_fusion_function_231017_2135:15' numobs=zeros(numveh,1);
  // 'EDGE_fusion_function_231017_2135:18' sourceconfig_ego=cell(1,numveh);
  // 'EDGE_fusion_function_231017_2135:19' for i= 1 : numveh
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231017_2135:20' sourceconfig_ego{i}=fuserSourceConfiguration(i);
  obj = &lobj_1[0];
  obj->pIsTransformToCentralValid = false;
  obj->pIsTransformToLocalValid = false;
  obj->SourceIndex = 1.0;
  obj->IsInternalSource = true;
  obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[0] = obj;

  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231017_2135:20' sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[1];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 2.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[1] = b_obj;

  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231017_2135:20' sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[2];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 3.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[2] = b_obj;

  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231017_2135:20' sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[3];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 4.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[3] = b_obj;

  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231017_2135:20' sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[4];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 5.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[4] = b_obj;

  // 'EDGE_fusion_function_231017_2135:23' fuser = trackFuser('SourceConfigurations',sourceconfig_ego,...  % 인지 융합 함수 configuration
  // 'EDGE_fusion_function_231017_2135:24'     'HasAdditiveProcessNoise',false,...
  // 'EDGE_fusion_function_231017_2135:25'     'AssignmentThreshold',[5 5],...
  // 'EDGE_fusion_function_231017_2135:26'     'ConfirmationThreshold',[5 5],...
  // 'EDGE_fusion_function_231017_2135:27'     'DeletionThreshold',[5 5],...
  // 'EDGE_fusion_function_231017_2135:28'     'Assignment','MatchPairs','StateFusion','Intersection');
  for (b_track_ego = 0; b_track_ego < 9; b_track_ego++) {
    fuser.ProcessNoise[b_track_ego] = b_iv[b_track_ego];
  }

  fuser.isInitialized = 0;
  b_obj = &fuser._pobj1[0];
  flag = (fuser.isInitialized == 1);
  if (flag) {
    fuser.TunablePropsChanged = true;
  }

  fuser.pSourceConfigurations[0] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[0]);
  fuser.pSourceConfigurations[1] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[1]);
  fuser.pSourceConfigurations[2] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[2]);
  fuser.pSourceConfigurations[3] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[3]);
  fuser.pSourceConfigurations[4] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[4]);
  fuser.pSourceConfigurations[5] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[5]);
  fuser.pSourceConfigurations[6] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[6]);
  fuser.pSourceConfigurations[7] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[7]);
  fuser.pSourceConfigurations[8] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[8]);
  fuser.pSourceConfigurations[9] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[9]);
  fuser.pSourceConfigurations[10] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[10]);
  fuser.pSourceConfigurations[11] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[11]);
  fuser.pSourceConfigurations[12] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[12]);
  fuser.pSourceConfigurations[13] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[13]);
  fuser.pSourceConfigurations[14] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[14]);
  fuser.pSourceConfigurations[15] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[15]);
  fuser.pSourceConfigurations[16] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[16]);
  fuser.pSourceConfigurations[17] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[17]);
  fuser.pSourceConfigurations[18] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[18]);
  fuser.pSourceConfigurations[19] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[19]);
  for (int i{0}; i < 5; i++) {
    fuser.pSourceConfigurations[i] = sourceconfig_ego[i];
  }

  fuser.pNumUsedConfigs = 5.0;
  std::memset(&ids[0], 0, 20U * sizeof(double));
  d = fuser.pNumUsedConfigs;
  b_track_ego = static_cast<int>(d);
  for (int i{0}; i < b_track_ego; i++) {
    ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
  }

  std::copy(&ids[0], &ids[20], &fuser.pSourceConfigIDs[0]);
  for (b_track_ego = 0; b_track_ego < 20; b_track_ego++) {
    fuser.pIsValidSource[b_track_ego] = false;
  }

  flag = (fuser.isInitialized == 1);
  if (flag) {
    fuser.TunablePropsChanged = true;
  }

  for (int i{0}; i < 5; i++) {
    fuser.pSourceConfigurations[i] = sourceconfig_ego[i];
  }

  fuser.pNumUsedConfigs = 5.0;
  std::memset(&ids[0], 0, 20U * sizeof(double));
  d = fuser.pNumUsedConfigs;
  b_track_ego = static_cast<int>(d);
  for (int i{0}; i < b_track_ego; i++) {
    ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
  }

  for (b_track_ego = 0; b_track_ego < 20; b_track_ego++) {
    fuser.pSourceConfigIDs[b_track_ego] = ids[b_track_ego];
    fuser.pIsValidSource[b_track_ego] = false;
  }

  fuser.matlabCodegenIsDeleted = false;

  //   % 인지 융합 함수 configuration
  // 'EDGE_fusion_function_231017_2135:30' for i = 1 : numveh
  // 'EDGE_fusion_function_231017_2135:34' idx=numobs>0;
  //  장애물 인지한 차량 인덱스
  // 'EDGE_fusion_function_231017_2135:35' numobs=numobs(idx);
  //  Obstacle state-space structure
  //  오류 수정 필요
  // 'EDGE_fusion_function_231017_2135:40' for i = 1 : numveh
  expl_temp.BranchID = 0U;
  expl_temp.Age = 1U;
  expl_temp.ObjectClassProbabilities = 1.0;
  expl_temp.TrackLogicState = true;
  expl_temp.IsConfirmed = true;
  expl_temp.IsCoasted = false;
  expl_temp.IsSelfReported = true;
  for (b_track_ego = 0; b_track_ego < 7; b_track_ego++) {
    expl_temp.TrackLogic[b_track_ego] = logicType[b_track_ego];
  }

  for (b_track_ego = 0; b_track_ego < 36; b_track_ego++) {
    expl_temp.StateCovariance[b_track_ego] = stateCov[b_track_ego];
  }

  for (int i{0}; i < 5; i++) {
    // 'EDGE_fusion_function_231017_2135:44' for j = 1 : numobs(i,1)
    expl_temp.SourceIndex = static_cast<unsigned int>(i + 1);

    // 'EDGE_fusion_function_231017_2135:48' obs_state =     [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231017_2135:49'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231017_2135:50'                  hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231017_2135:51'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231017_2135:52'                  hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231017_2135:53'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231017_2135:55' temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231017_2135:56'                            'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231017_2135:57'                            'SourceIndex',i, ...
    // 'EDGE_fusion_function_231017_2135:58'                            'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231017_2135:59'                            'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp, ...
    // 'EDGE_fusion_function_231017_2135:60'                            'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[0].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[0].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[0].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[0].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[0].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[0].Velocity_z;

    // 'EDGE_fusion_function_231017_2135:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 1);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[0].timestamp;
    expl_temp.ObjectClassID = hub_data_Object->vehicle[i].obstacle[0].
      obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;

    // 'EDGE_fusion_function_231017_2135:48' obs_state =     [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231017_2135:49'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231017_2135:50'                  hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231017_2135:51'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231017_2135:52'                  hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231017_2135:53'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231017_2135:55' temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231017_2135:56'                            'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231017_2135:57'                            'SourceIndex',i, ...
    // 'EDGE_fusion_function_231017_2135:58'                            'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231017_2135:59'                            'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp, ...
    // 'EDGE_fusion_function_231017_2135:60'                            'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[1].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[1].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[1].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[1].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[1].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[1].Velocity_z;

    // 'EDGE_fusion_function_231017_2135:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 2);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[1].timestamp;
    expl_temp.ObjectClassID = hub_data_Object->vehicle[i].obstacle[1].
      obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;

    // 'EDGE_fusion_function_231017_2135:48' obs_state =     [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231017_2135:49'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231017_2135:50'                  hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231017_2135:51'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231017_2135:52'                  hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231017_2135:53'                  hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231017_2135:55' temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231017_2135:56'                            'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231017_2135:57'                            'SourceIndex',i, ...
    // 'EDGE_fusion_function_231017_2135:58'                            'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231017_2135:59'                            'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp, ...
    // 'EDGE_fusion_function_231017_2135:60'                            'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[2].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[2].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[2].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[2].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[2].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[2].Velocity_z;

    // 'EDGE_fusion_function_231017_2135:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 3);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[2].timestamp;
    expl_temp.ObjectClassID = hub_data_Object->vehicle[i].obstacle[2].
      obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;
  }

  // 'EDGE_fusion_function_231017_2135:67' [confirmedTracks,tentativeTracks,allTracks,analysisInformation]=fuser(track_ego,hub_data_Object.vehicle(1).obstacle(1).timestamp);
  confirmedTracks_size[0] = fuser.step(track_ego, hub_data_Object->vehicle[0].
    obstacle[0].timestamp, confirmedTracks_data, tentativeTracks_data, allTracks,
    analysisInformation->TrackIDsAtStepBeginning.data,
    analysisInformation->TrackIDsAtStepBeginning.size,
    analysisInformation->CostMatrix, analysisInformation->Assignments,
    analysisInformation->UnassignedCentralTracks,
    analysisInformation->UnassignedLocalTracks,
    analysisInformation->NonInitializingLocalTracks,
    analysisInformation->InitializedCentralTrackIDs.data,
    analysisInformation->InitializedCentralTrackIDs.size,
    analysisInformation->UpdatedCentralTrackIDs,
    analysisInformation->DeletedTrackIDs.data,
    analysisInformation->DeletedTrackIDs.size,
    analysisInformation->TrackIDsAtStepEnd.data,
    analysisInformation->TrackIDsAtStepEnd.size, tentativeTracks_size[0]);
}

//
// Arguments    : void
// Return Type  : void
//
void EDGE_fusion_function_231017_2135_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void EDGE_fusion_function_231017_2135_terminate()
{
}

//
// File trailer for EDGE_fusion_function_231017_2135.cpp
//
// [EOF]
//
