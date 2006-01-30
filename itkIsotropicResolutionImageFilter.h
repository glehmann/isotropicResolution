/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkIsotropicResolutionImageFilter.h,v $
  Language:  C++
  Date:      $Date: 2005/04/04 14:37:29 $
  Version:   $Revision: 1.1 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkIsotropicResolutionImageFilter_h
#define __itkIsotropicResolutionImageFilter_h

#include <vector>
#include <queue>
#include "itkImageToImageFilter.h"
#include "itkImage.h"
#include "itkNumericTraits.h"
#include "itkConceptChecking.h"
#include "itkInterpolateImageFunction.h"

namespace itk
{
/**
 * \class IsotropicResolutionImageFilter
 * \brief Grow or shrink the input image to create a new image with isotropic resolution
 *
 * IsotropicResolutionImageFilter is a convenient filter to get a image with isotropic
 * resolution from an image with anisotropic resolution. It increase the size of the
 * input image to get the same spacing on each axes.
 * IsotropicResolutionImageFilter is far less powerful than ResampleImageFilter (even
 * if it use it internally), but is also far much simple to use and to integrate in 
 * a pipeline. If you need to fine tune the settings to resample your image, you should
 * prefer ResampleImageFilter.
 *
 * You can set a maximum increase of number of pixel on each axe. If the maximum is
 * reached IsotropicResolutionImageFilter will set the spacing on the axe to be the
 * maximum authorized, and will decrease the number of pixels on the other axes to 
 * keep an isotropic resolution on output image.
 * 
 * Note that the choice of interpolator function can be important.
 * This function is set via SetNearestNeighbor().  The default is
 * false, which mean a itk::LinearInterpolateImageFunction<InputImageType,
 * TInterpolatorPrecisionType> is used and that axe which are loosing
 * pixels are smooth with a itk::RecursiveGaussianImageFilter. SetNearestNeighbor(false)
 * is reasonable for ordinary medical images.  However, some synthetic
 * images have pixels drawn from a finite prescribed set.  An example
 * would be a mask indicating the segmentation of a brain into a small
 * number of tissue types.  For such an image, one does not want to
 * interpolate between different pixel values, and so SetNearestNeighbor(true)
 * should be used. In that case, a itk::NearestNeighborInterpolateImageFunction< InputImageType,
 * TInterpolatorPrecisionType > is used internally and no smoothing is done.
 *
 * \author Gaëtan Lehmann. Biologie du Développement et de la Reproduction, INRA de Jouy-en-Josas, France.
 *
 * \sa ResampleImageFilter
 */
template <class TInputImage, class TOutputImage, class TInterpolatorPrecisionType=double>
class ITK_EXPORT IsotropicResolutionImageFilter :
    public ImageToImageFilter< TInputImage, TOutputImage >
{
public:
  /** Extract dimension from input and output image. */
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TOutputImage::ImageDimension);

  /** Convenient typedefs for simplifying declarations. */
  typedef TInputImage InputImageType;
  typedef TOutputImage OutputImageType;

  /** Standard class typedefs. */
  typedef IsotropicResolutionImageFilter Self;
  typedef ImageToImageFilter< InputImageType, OutputImageType> Superclass;
  typedef SmartPointer<Self> Pointer;
  typedef SmartPointer<const Self>  ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(IsotropicResolutionImageFilter, ImageToImageFilter);

  /** Image typedef support. */
  typedef typename InputImageType::PixelType InputPixelType;
  typedef typename OutputImageType::PixelType OutputPixelType;
  typedef typename NumericTraits<InputPixelType>::RealType InputRealType;
  typedef typename InputImageType::OffsetType OffsetType;
  typedef typename InputImageType::IndexType IndexType;

  typedef typename InputImageType::RegionType InputImageRegionType;
  typedef typename OutputImageType::RegionType OutputImageRegionType;
  typedef typename InputImageType::SizeType InputSizeType;

  /** Set/Get wether the interpolator must be a nearest neighbor or not.
   * If false, the interpolator is a linear interpolator.
   * Default is false.
   */
  itkSetMacro(NearestNeighbor, bool);
  itkGetConstReferenceMacro(NearestNeighbor, bool);
  itkBooleanMacro(NearestNeighbor);

  /** Set the maximum increase spacing factor. Default is 0, which means no limit. */
  itkSetMacro(MaximumIncrease, double);

  /** Get the maximum increase factor. */
  itkGetMacro(MaximumIncrease, double);

protected:
  IsotropicResolutionImageFilter();
  virtual ~IsotropicResolutionImageFilter(){}
  void PrintSelf(std::ostream& os, Indent indent) const;

  virtual void GenerateInputRequestedRegion() ;
  
  virtual void GenerateOutputInformation();
  
  void  GenerateData ();
  
private:
  IsotropicResolutionImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  
  bool m_NearestNeighbor;
  
  double m_MaximumIncrease;

};

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkIsotropicResolutionImageFilter.txx"
#endif

#endif
