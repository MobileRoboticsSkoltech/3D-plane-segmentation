// Copyright (c) 2008  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/GraphicsView/include/CGAL/Qt/GraphicsItem.h $
// $Id: GraphicsItem.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Andreas Fabri <Andreas.Fabri@geometryfactory.com>
//                 Laurent Rineau <Laurent.Rineau@geometryfactory.com>

#ifndef CGAL_QT_GRAPHICS_ITEM_H
#define CGAL_QT_GRAPHICS_ITEM_H

#include <CGAL/license/GraphicsView.h>


#include <CGAL/export/Qt.h>
#include <CGAL/auto_link/Qt.h>

#include <QObject>
#include <QGraphicsItem>
#ifndef Q_MOC_RUN
#  include <CGAL/Object.h>
#endif



namespace CGAL {
namespace Qt {

class CGAL_QT_EXPORT GraphicsItem : public QObject, public QGraphicsItem {

  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public Q_SLOTS:

  virtual void modelChanged() = 0;
};


} // namespace Qt
} // namespace CGAL

#endif // CGAL_QT_GRAPHICS_ITEM_H
