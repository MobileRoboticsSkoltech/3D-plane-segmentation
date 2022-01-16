// Copyright (c) 2008  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/GraphicsView/include/CGAL/Qt/utility_impl.h $
// $Id: utility_impl.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Andreas Fabri <Andreas.Fabri@geometryfactory.com>
//                 Laurent Rineau <Laurent.Rineau@geometryfactory.com>
   
#ifdef CGAL_HEADER_ONLY
#define CGAL_INLINE_FUNCTION inline

#include <CGAL/license/GraphicsView.h>

#else
#define CGAL_INLINE_FUNCTION
#endif

#include <CGAL/Qt/utility.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QList>
#include <QPoint>
#include <QPointF>

namespace CGAL {
namespace Qt {

CGAL_INLINE_FUNCTION
QRectF mapToScene(const QGraphicsView* v, const QRect rect)
{
  QPointF top_left = v->mapToScene(rect.topLeft());
  QPointF size = v->mapToScene(rect.bottomRight());
  size -= top_left;
  return QRectF(top_left.x(),
		top_left.y(),
		size.x(),
		size.y());
}

CGAL_INLINE_FUNCTION
QRectF viewportsBbox(const QGraphicsScene* scene) {
   QRectF rect;
   Q_FOREACH(QGraphicsView* view, scene->views())
   {
     rect |= mapToScene(view, view->viewport()->rect());
   }
   rect = rect.normalized();
   return rect;
}

} // namespace Qt
} // namespace CGAL
