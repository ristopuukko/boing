/////////////////////////////////////////////////////////////////////////////
// Name:        wx/gtk/menu.h
// Purpose:
// Author:      Robert Roebling
// Id:          $Id: menu.h 54123 2008-06-11 17:07:07Z PC $
// Copyright:   (c) 1998 Robert Roebling, Julian Smart
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

#ifndef _WX_GTKMENU_H_
#define _WX_GTKMENU_H_

//-----------------------------------------------------------------------------
// wxMenuBar
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_CORE wxMenuBar : public wxMenuBarBase
{
public:
    // ctors
    wxMenuBar();
    wxMenuBar(long style);
    wxMenuBar(size_t n, wxMenu *menus[], const wxString titles[], long style = 0);
    virtual ~wxMenuBar();

    // implement base class (pure) virtuals
    virtual bool Append( wxMenu *menu, const wxString &title );
    virtual bool Insert(size_t pos, wxMenu *menu, const wxString& title);
    virtual wxMenu *Replace(size_t pos, wxMenu *menu, const wxString& title);
    virtual wxMenu *Remove(size_t pos);

    virtual int FindMenuItem(const wxString& menuString,
                             const wxString& itemString) const;
    virtual wxMenuItem* FindItem( int id, wxMenu **menu = NULL ) const;

    virtual void EnableTop( size_t pos, bool flag );
    virtual void SetMenuLabel( size_t pos, const wxString& label );
    virtual wxString GetMenuLabel( size_t pos ) const;

    void SetLayoutDirection(wxLayoutDirection dir);
    wxLayoutDirection GetLayoutDirection() const;

    // wxMenuBar is not a top level window but it still doesn't need a parent
    // window
    virtual bool GTKNeedsParent() const { return false; }

    void Attach(wxFrame *frame);

    // implementation only from now on
    void SetInvokingWindow( wxWindow *win );
    void UnsetInvokingWindow( wxWindow *win );

private:
    // common part of Append and Insert
    bool GtkAppend(wxMenu *menu, const wxString& title, int pos=-1);

    GtkWidget       *m_menubar;
    wxWindow        *m_invokingWindow;

    void Init(size_t n, wxMenu *menus[], const wxString titles[], long style);

    DECLARE_DYNAMIC_CLASS(wxMenuBar)
};

//-----------------------------------------------------------------------------
// wxMenu
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_CORE wxMenu : public wxMenuBase
{
public:
    // ctors & dtor
    wxMenu(const wxString& title, long style = 0)
        : wxMenuBase(title, style) { Init(); }

    wxMenu(long style = 0) : wxMenuBase(style) { Init(); }

    virtual ~wxMenu();

    void Attach(wxMenuBarBase *menubar);

    void SetLayoutDirection(const wxLayoutDirection dir);
    wxLayoutDirection GetLayoutDirection() const;

    // TODO: virtual void SetTitle(const wxString& title);

    // implementation GTK only
    GtkWidget       *m_menu;  // GtkMenu
    GtkWidget       *m_owner;
    GtkAccelGroup   *m_accel;
    bool m_popupShown;

protected:
    virtual wxMenuItem* DoAppend(wxMenuItem *item);
    virtual wxMenuItem* DoInsert(size_t pos, wxMenuItem *item);
    virtual wxMenuItem* DoRemove(wxMenuItem *item);

private:
    // common code for all constructors:
    void Init();

    // common part of Append (if pos == -1)  and Insert
    bool GtkAppend(wxMenuItem *item, int pos=-1);

    GtkWidget *m_prevRadio;

    DECLARE_DYNAMIC_CLASS(wxMenu)
};

#endif
    // _WX_GTKMENU_H_
