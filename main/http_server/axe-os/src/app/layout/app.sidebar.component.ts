import { Component, ElementRef, ChangeDetectionStrategy } from '@angular/core';
import { LayoutService } from "./service/app.layout.service";

@Component({
    selector: 'app-sidebar',
    templateUrl: './app.sidebar.component.html',
    changeDetection: ChangeDetectionStrategy.Eager,
    standalone: false
})
export class AppSidebarComponent {
    constructor(public layoutService: LayoutService, public el: ElementRef) { }
}

