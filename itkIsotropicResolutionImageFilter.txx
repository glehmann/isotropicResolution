/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkIsotropicResolutionImageFilter.txx,v $
  Language:  C++
  Date:      $Date: 2005/08/23 15:56:54 $
  Version:   $Revision: 1.2 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef _itkIsotropicResolutionImageFilter_txx
#define _itkIsotropicResolutionImageFilter_txx

#include "itkResampleImageFilter.h"
#include <itkLinearInterpolateImageFunction.h>
#include "itkProgressAccumulator.h"

namespace itk
{
        
template <class TInputImage, class TOutputImage, class TInterpolatorPrecisionType>
IsotropicResolutionImageFilter<TInputImage, TOutputImage, TInterpolatorPrecisionType>
::IsotropicResolutionImageFilter()
{
  m_Interpolator = LinearInterpolateImageFunction<InputImageType, TInterpolatorPrecisionType>::New();
  m_MaximumIncrease = 0;
}

template <class TInputImage, class TOutputImage, class TInterpolatorPrecisionType>
void 
IsotropicResolutionImageFilter<TInputImage, TOutputImage, TInterpolatorPrecisionType>
::GenerateInputRequestedRegion()
{
  // call the superclass's implementation of this method
  Superclass::GenerateInputRequestedRegion();

  if ( !this->GetInput() )
    {
    return;
    }

  // get pointers to the input and output
  typename InputImageType::Pointer  inputPtr  = 
    const_cast< TInputImage *>( this->GetInput() );

  // Request the entire input image
  InputImageRegionType inputRegion;
  inputRegion = inputPtr->GetLargestPossibleRegion();
  inputPtr->SetLargestPossibleRegion(inputRegion);
  inputPtr->SetRequestedRegion(inputRegion);

  return;
}

/** 
 * Inform pipeline of required output region
 */
template <class TInputImage, class TOutputImage, class TInterpolatorPrecisionType>
void 
IsotropicResolutionImageFilter<TInputImage, TOutputImage, TInterpolatorPrecisionType>
::GenerateOutputInformation()
{
  // call the superclass' implementation of this method
  Superclass::GenerateOutputInformation();

  // get pointers to the input and output
  typename OutputImageType::Pointer outputPtr = this->GetOutput();
  if ( !outputPtr )
    {
    return;
    }
    
  typename InputImageType::SpacingType inputSpacing = this->GetInput()->GetSpacing();
  typename InputImageType::SpacingType isoSpacing;
  
  typename InputImageType::SizeType inputSize = this->GetInput()->GetLargestPossibleRegion().GetSize();
  typename OutputImageType::SizeType outputSize;
  
  // find the smallest spacing
  double smallestSpacing = NumericTraits< double >::max();
  double biggestSpacing = 0;
  for( int i=0; i < InputImageDimension; i++)
    {
    if ( smallestSpacing > inputSpacing[i] )
      {
      smallestSpacing = inputSpacing[i];
      }
    if ( biggestSpacing < inputSpacing[i] )
      {
      biggestSpacing = inputSpacing[i];
      }
    }
    
  if ( m_MaximumIncrease != 0 && biggestSpacing / smallestSpacing > m_MaximumIncrease )
    {
    isoSpacing.Fill( biggestSpacing / m_MaximumIncrease );
    }
  else
    {
    // use the smallest spacing on all axes
    isoSpacing.Fill( smallestSpacing );
    }
    
  // output size
  for( int i=0; i < InputImageDimension; i++)
    {
      if( inputSpacing[i] == isoSpacing[i] )
        {
        outputSize[i] = inputSize[i];
        }
      else
        {
        outputSize[i] = static_cast<unsigned long>( (inputSize[i] - 1) * inputSpacing[i] / isoSpacing[i] + 1);
        }
    }
  
  outputPtr->SetLargestPossibleRegion( outputSize );
  outputPtr->SetSpacing( isoSpacing );
  outputPtr->SetOrigin( this->GetInput()->GetOrigin() );
  outputPtr->SetDirection( this->GetInput()->GetDirection() );
  
  return;
}

        

template< class TInputImage, class TOutputImage, class TInterpolatorPrecisionType>
void
IsotropicResolutionImageFilter< TInputImage, TOutputImage, TInterpolatorPrecisionType>
::GenerateData( void )
{
  // Allocate the outputs
  this->AllocateOutputs();

  // input spacing
  typename InputImageType::SpacingType inputSpacing = this->GetInput()->GetSpacing();
  
  // output spacing and size
  typename InputImageType::SpacingType isoSpacing = this->GetOutput()->GetSpacing();
  typename OutputImageType::SizeType outputSize = this->GetOutput()->GetLargestPossibleRegion().GetSize();
  
  /** set up resample image filter */
  typename ResampleImageFilter<TInputImage, TOutputImage >::Pointer
    resample = ResampleImageFilter<TInputImage, TOutputImage, TInterpolatorPrecisionType>::New();

  resample->SetInput( this->GetInput() );
  resample->ReleaseDataFlagOn();
  resample->SetOutputSpacing( isoSpacing );
  resample->SetSize( outputSize );
  resample->SetInterpolator( m_Interpolator );
  
  /** set up the minipipeline */
  typename ProgressAccumulator::Pointer progress = ProgressAccumulator::New();
  progress->SetMiniPipelineFilter(this);
  progress->RegisterInternalFilter(resample, 1.0f);
  
  resample->GraftOutput( this->GetOutput() );

  /** execute the minipipeline */
  resample->Update();

  /** graft the minipipeline output back into this filter's output */
  this->GraftOutput( resample->GetOutput() );
}


/**
 * Standard "PrintSelf" method
 */
template <class TInputImage, class TOutputImage, class TInterpolatorPrecisionType>
void
IsotropicResolutionImageFilter<TInputImage, TOutputImage, TInterpolatorPrecisionType>
::PrintSelf( std::ostream& os, Indent indent) const
{
  Superclass::PrintSelf( os, indent );
  os << indent << "Interpolator: " << std::endl;
  m_Interpolator->Print(os, indent.GetNextIndent());
  os << indent << "MaximumIncrease: " << m_MaximumIncrease << std::endl;
}

} // end namespace itk

#endif
